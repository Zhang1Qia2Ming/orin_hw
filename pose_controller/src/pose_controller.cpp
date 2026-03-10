#include "pose_controller/pose_controller.hpp"
#include "sensor_base/data_layouts.hpp"

#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

namespace pose_controller {
    PoseController::PoseController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn PoseController::on_init() {
        try {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PoseController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // get params
        params_ = param_listener_->get_params();

        for (const auto& interface_name : params_.pose_list) {
            // add pose stream context
            auto pose_stream_context = std::make_shared<PoseStreamContext>();
            pose_stream_context->interface_name = interface_name;
            pose_stream_context->topic_name = interface_name;

            pose_stream_context->odom_pub_ = get_node()->create_publisher<nav_msgs::msg::Odometry>(
                pose_stream_context->topic_name, 10);
            
            pose_stream_context->queue_ = std::queue<PosePublishTask>();

            pose_stream_contexts_.push_back(pose_stream_context);

            RCLCPP_INFO(get_node()->get_logger(), "Configured stream: %s", interface_name.c_str());
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn PoseController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        for(auto& ctx : pose_stream_contexts_) {
            ctx->last_update_count = 0;
            ctx->thread_ = std::thread(&PoseController::worker_thread, this, ctx);
        }

        RCLCPP_INFO(get_node()->get_logger(), "Pose stream activated ! %zu worker threads started.", pose_stream_contexts_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration PoseController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto& pose_stream_context : pose_stream_contexts_) {
            config.names.push_back(pose_stream_context->interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration PoseController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::CallbackReturn PoseController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = false;

        for(auto& ctx : pose_stream_contexts_) {
            ctx->cv_.notify_all();
            if(ctx->thread_.joinable()) {
                ctx->thread_.join();
            }

            std::queue<PosePublishTask> empty_queue;
            {
                std::lock_guard<std::mutex> lock(ctx->mutex_);
                std::swap(ctx->queue_, empty_queue);
            }
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type PoseController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        
        if(state_interfaces_.size() != pose_stream_contexts_.size()) {
            return controller_interface::return_type::OK;
        }
        // RCLCPP_INFO(get_node()->get_logger(), "update %zu pose streams!!!", pose_stream_contexts_.size());
        for(size_t i = 0;i< pose_stream_contexts_.size();i++) {
            auto ctx = pose_stream_contexts_[i];

            // double ptr_value = state_interfaces_[i].get_value();

            double ptr_value = 0.0;
            bool found_interface = false;

            // shield
            for (const auto& interface : state_interfaces_) {
                if (interface.get_name() == ctx->interface_name) {
                    try {
                        ptr_value = interface.get_value();
                        found_interface = true;
                        break; 
                    } catch (const std::exception& e) {
                        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "[%s] HW interface error: %s", ctx->interface_name.c_str(), e.what());
                        break;
                    }
                }
            }

            if(ptr_value == 0.0) {
                RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "[%s] HW interface error: ptr_value is 0.0", ctx->interface_name.c_str());
            }


            if(!found_interface || std::isnan(ptr_value) || ptr_value == 0.0) {
                continue;
            }

            sensor_base::PoseDataLayout* get_pose_ptr = nullptr;
            std::memcpy(&get_pose_ptr, &ptr_value, sizeof(get_pose_ptr));

            if(get_pose_ptr == nullptr) {
                continue;
            }

            if(get_pose_ptr->header.update_count > ctx->last_update_count) {
                
                //update local
                ctx->last_update_count = get_pose_ptr->header.update_count;

                PosePublishTask task;
                task.timestamp_nanos = get_pose_ptr->header.timestamp_nanos;
                task.update_count = get_pose_ptr->header.update_count;
                task.pose = *get_pose_ptr;

                {
                    std::lock_guard<std::mutex> lock(ctx->mutex_);
                    ctx->queue_.push(task);

                    if(ctx->queue_.size() > 10) {
                        ctx->queue_.pop();
                    }                   
                }
                ctx->cv_.notify_one();
            } else {
                // RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                //             "[%s] HW interface error: update_count is not increasing", ctx->interface_name.c_str());
            }
        }

        return controller_interface::return_type::OK;
    }

    void PoseController::worker_thread(std::shared_ptr<PoseStreamContext> ctx) {
        while(is_running_) {
            
            PosePublishTask task;
            {
                std::unique_lock<std::mutex> lock(ctx->mutex_);
                ctx->cv_.wait(lock, [&ctx, this] {
                    return !ctx->queue_.empty() || !is_running_;
                });

                if(!is_running_ && ctx->queue_.empty()) {
                    break;
                }
                task = ctx->queue_.front();
                ctx->queue_.pop();
            }


            std_msgs::msg::Header header;
            header.stamp.sec = task.timestamp_nanos / 1000000000ULL;
            header.stamp.nanosec = task.timestamp_nanos % 1000000000ULL;

            header.frame_id = "odom";
            std::string child_frame_id = ctx->interface_name.substr(0, ctx->interface_name.find("/"));

            auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
            odom_msg->header = header;
            odom_msg->child_frame_id = child_frame_id;

            odom_msg->pose.pose.position.x = task.pose.pose[0];
            odom_msg->pose.pose.position.y = task.pose.pose[1];
            odom_msg->pose.pose.position.z = task.pose.pose[2];
            odom_msg->pose.pose.orientation.x = task.pose.pose[3];
            odom_msg->pose.pose.orientation.y = task.pose.pose[4];
            odom_msg->pose.pose.orientation.z = task.pose.pose[5];
            odom_msg->pose.pose.orientation.w = task.pose.pose[6];

            odom_msg->twist.twist.linear.x = task.pose.velocity[0];
            odom_msg->twist.twist.linear.y = task.pose.velocity[1];
            odom_msg->twist.twist.linear.z = task.pose.velocity[2];
            odom_msg->twist.twist.angular.x = task.pose.velocity[3];
            odom_msg->twist.twist.angular.y = task.pose.velocity[4];
            odom_msg->twist.twist.angular.z = task.pose.velocity[5];

            try {
                // publish odom message
                ctx->odom_pub_->publish(*odom_msg);
            } catch(const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "Odometry publish exception: %s", e.what());
            }
        }
    }
} // namespace pose_controller

