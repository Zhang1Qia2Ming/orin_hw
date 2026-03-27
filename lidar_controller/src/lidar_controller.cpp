#include "lidar_controller/lidar_controller.hpp"
#include "sensor_base/data_layouts.hpp"

#include "pthread.h"

namespace lidar_controller {
    LidarController::LidarController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn LidarController::on_init() {
        try {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LidarController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // get params
        params_ = param_listener_->get_params();

        for(const auto& interface_name : params_.lidar_list) {
            RCLCPP_INFO(get_node()->get_logger(), "interface_name: %s", interface_name.c_str());
            auto lidar_stream_context = std::make_shared<LidarStreamContext>();
            lidar_stream_context->interface_name = interface_name;
            lidar_stream_context->topic_name = interface_name;
            lidar_stream_context->frame_id = params_.lidar_config.lidar_list_map.find(interface_name)->second.frame_id;

            lidar_stream_context->lidar_livox_pub_ = get_node()->create_publisher<CustomMsg>(
                lidar_stream_context->topic_name,
                rclcpp::SensorDataQoS());

            lidar_stream_context->lidar_point_cloud_pub_ = get_node()->create_publisher<PointCloud2>(
                lidar_stream_context->topic_name + "_point_cloud",
                rclcpp::SensorDataQoS());

            if(params_.lidar_config.lidar_list_map.find(interface_name)->second.enable_dynamic_extrinsics) {
                lidar_stream_context->enable_dynamic_extrinsics = true;

                auto transform_callback = [ctx = lidar_stream_context](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                    Eigen::Translation3f translation(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
                    Eigen::Quaternionf rotation(msg->pose.orientation.w, msg->pose.orientation.x, 
                        msg->pose.orientation.y, msg->pose.orientation.z);
                    
                    Eigen::Affine3f new_extrinsics = translation * rotation;

                    {
                        std::lock_guard<std::mutex> lock(ctx->extrinsics_mutex_);
                        ctx->current_extrinsics = new_extrinsics;
                        ctx->extrinsics_updated_ = true;
                    }
                };

                lidar_stream_context->dynamic_livox_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
                    lidar_stream_context->interface_name + "/extrinsics",10, transform_callback);
            } else {
                lidar_stream_context->enable_dynamic_extrinsics = false;
                lidar_stream_context->dynamic_livox_sub_ = nullptr;
            }

            lidar_stream_contexts_.push_back(lidar_stream_context);
        }
        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LidarController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        for(auto& ctx : lidar_stream_contexts_) {
            for(int i = 0; i < 8; i++) {
                ctx->free_queue_.push(i);
            }
            ctx->publish_tasks_pool_.resize(8);
            for(auto& task : ctx->publish_tasks_pool_) {
                task.raw_data.points.reserve(24000);
            }
            ctx->thread_ = std::thread(&LidarController::publish_worker, this, ctx);
        }
        
        RCLCPP_INFO(get_node()->get_logger(), "Lidar stream activated ! with %zu threads", lidar_stream_contexts_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration LidarController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto& ctx : lidar_stream_contexts_) {
            config.names.push_back(ctx->interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration LidarController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::CallbackReturn LidarController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = false;

        for(auto& ctx : lidar_stream_contexts_) {
            // todo: join thread
            if(ctx->thread_.joinable()) {
                ctx->thread_.join();
            }
        }

        RCLCPP_INFO(get_node()->get_logger(), "Lidar stream deactivated ! with %zu threads", lidar_stream_contexts_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type LidarController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        
        // get tri buffer pointer
        
        // if data is updated,get new data block from lidar pool
        // deep copy here
        // then ,give index to work queue
        for(size_t i = 0; i < lidar_stream_contexts_.size(); i++) {
            auto& ctx = lidar_stream_contexts_[i];

            double ptr_value = 0.0;
            bool found_interface = false;

            for (const auto& interface : state_interfaces_) {
                if (interface.get_name() == ctx->interface_name) {
                    try {
                        // 🌟 2. 防弹衣：接住 get_value 可能抛出的致命异常！
                        ptr_value = interface.get_value();
                        found_interface = true;
                        break; 
                    } catch (const std::exception& e) {
                        // 如果底层传来的是 nullptr，异常会被这里吃掉，只打印警告，绝不崩溃！
                        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                            "[%s] HW interface error: %s", ctx->interface_name.c_str(), e.what());
                        break;
                    }
                }
            }

            if(!found_interface || std::isnan(ptr_value) || ptr_value == 0.0) {
                continue;
            }

            if(ptr_value != ctx->last_ptr_value) {
                
                // debug for see memory address
                // RCLCPP_INFO(get_node()->get_logger(), "ptr_value: %p", 
                //     *reinterpret_cast<void**>(&ptr_value));
                ctx->last_ptr_value = ptr_value;
                int index = -1;
                if(ctx->free_queue_.pop(index)) {
                    sensor_base::LidarDataLayout* raw_ptr = *reinterpret_cast<sensor_base::LidarDataLayout**>(&ptr_value);
                    // FillLidarPublishTaskWithPoints(ctx->publish_tasks_pool_[index], *raw_ptr);

                    // auto timestamp_0 = std::chrono::system_clock::now();
                    
                    // ctx->publish_tasks_pool_[index].raw_data = *raw_ptr;
                    auto& dest = ctx->publish_tasks_pool_[index].raw_data;
                    dest.point_num = raw_ptr->point_num;
                    dest.points.assign(raw_ptr->points.begin(), raw_ptr->points.begin() + raw_ptr->point_num);
                    
                    // FillLidarPublishTaskWithPoints2(ctx->publish_tasks_pool_[index], *raw_ptr);
                    // auto timestamp_1 = std::chrono::system_clock::now();
                    // RCLCPP_INFO(get_node()->get_logger(), "FillLidarPublishTaskWithPoints2 cost: %ld us", std::chrono::duration_cast<std::chrono::microseconds>(timestamp_1 - timestamp_0).count());

                    if(ctx->work_queue_.push(index)) {
                        // RCLCPP_INFO(get_node()->get_logger(), "work_queue's occupancy/all: %ld/%ld", ctx->work_queue_.size(), ctx->work_queue_.max_size());
                    } else {
                        RCLCPP_WARN(get_node()->get_logger(), "work_queue is full, drop data");
                    }
                } else {
                    RCLCPP_WARN(get_node()->get_logger(), "free_queue is empty, drop data");
                }
                
            } 
        }
        return controller_interface::return_type::OK;
    }

    void LidarController::publish_worker(std::shared_ptr<LidarStreamContext> ctx) {
        RCLCPP_INFO(get_node()->get_logger(), "Lidar stream %s publish worker started", ctx->interface_name.c_str());
        while(is_running_) {
            int index = -1;
            if(ctx->work_queue_.pop(index)) {
                // ctx->lidar_livox_pub_->publish(ctx->publish_tasks_pool_[index].msg);
                Eigen::Affine3f current_transform = Eigen::Affine3f::Identity();
                bool do_transform = false;

                if(ctx->enable_dynamic_extrinsics) {
                    std::lock_guard<std::mutex> lock(ctx->extrinsics_mutex_);
                    if(ctx->extrinsics_updated_) {
                        current_transform = ctx->current_extrinsics;
                        do_transform = true;

                        // print current_transform:Eigen::Affine3f every member
                        // RCLCPP_INFO(get_node()->get_logger(), "current_transform: %f, %f, %f", current_transform.translation().x(), current_transform.translation().y(), current_transform.translation().z());
                    }
                }
                FillLidarPublishTaskWithPoints2(ctx->publish_tasks_pool_[index], 
                                                ctx->publish_tasks_pool_[index].raw_data,
                                                do_transform,
                                                current_transform,
                                                ctx
                                            );
                
                // debug for see payload kb and mb
                // auto& msg = ctx->publish_tasks_pool_[index].point_cloud_msg;
                // size_t payload_bytes = msg.data.size();
                // double payload_kb = payload_bytes / 1024.0;
                // double payload_mb = payload_kb / 1024.0;
                // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                //     "Lidar stream %s publish payload kb: %f, payload mb: %f", ctx->interface_name.c_str(), payload_kb, payload_mb);
                
                // publish point cloud
                ctx->lidar_point_cloud_pub_->publish(ctx->publish_tasks_pool_[index].point_cloud_msg);


                // static auto last_pub_time = std::chrono::system_clock::now();
                // auto now = std::chrono::system_clock::now();
                // double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last_pub_time).count() / 1000000.0;
                // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                //     "Lidar stream %s publish cost: %f s", ctx->interface_name.c_str(), dt);
                // last_pub_time = now;
                ctx->free_queue_.push(index);
            } else {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        RCLCPP_INFO(get_node()->get_logger(), "Lidar stream %s publish worker stopped", ctx->interface_name.c_str());
    }

    void LidarController::FillLidarPublishTaskWithPoints(LidarPublishTask& task, const sensor_base::LidarDataLayout& data) 
    {
        uint32_t points_num = data.point_num;

        if(points_num == 0) {
            return;
        }

        const std::vector<sensor_base::LidarDataPointLayout> &points = data.points;
        task.timestamp_nanos = data.points[0].offset_time;

        task.msg.points.clear();

        task.msg.points.reserve(points_num);
        // RCLCPP_INFO(rclcpp::get_logger("debug!!!"), "Lidar stream publish points num: %d", points_num);
        for(uint32_t i = 0; i < points_num; ++i) {
            CustomPoint point;
            point.x = points[i].x;
            point.y = points[i].y;
            point.z = points[i].z;
            point.reflectivity = points[i].intensity;
            point.tag = points[i].tag;
            point.line = points[i].line;
            point.offset_time =
                static_cast<uint32_t>(points[i].offset_time - task.timestamp_nanos);

            task.msg.points.push_back(std::move(point));
        }

    }

    void LidarController::FillLidarPublishTaskWithPoints2(  LidarPublishTask& task, 
                                                            const sensor_base::LidarDataLayout& data,
                                                            bool do_transform,
                                                            const Eigen::Affine3f& transform,
                                                            std::shared_ptr<LidarStreamContext> ctx) 
    {
        uint32_t points_num = data.point_num;
        if(points_num == 0) {
            return;
        }

        const auto &points = data.points;
        task.timestamp_nanos = points[0].offset_time;
        
        auto& msg = task.point_cloud_msg;
        msg.header.stamp = rclcpp::Time(task.timestamp_nanos);
        msg.header.frame_id = ctx->frame_id; 
        msg.width = points_num;
        msg.height = 1;
        msg.is_bigendian = false;
        msg.is_dense = true;
        msg.point_step = sizeof(sensor_base::LidarDataPointLayout);
        msg.row_step = msg.point_step * points_num;

        // 🌟 护城河 1：动态配置点云字段解析规则 (利用 offsetof 宏实现内存级别的精准对齐)
        // 注意：这只会执行一次，因为复用池里的对象 fields 填过一次就不为空了
        if (msg.fields.empty()) {
            auto add_field = [&](const std::string& name, uint32_t offset, uint8_t datatype) {
                sensor_msgs::msg::PointField f;
                f.name = name;
                f.offset = offset;
                f.datatype = datatype;
                f.count = 1;
                msg.fields.push_back(f);
            };

            // 注意：请根据你 LidarDataPointLayout 里的真实类型调整 FLOAT32 还是 UINT8！
            add_field("x", offsetof(sensor_base::LidarDataPointLayout, x), sensor_msgs::msg::PointField::FLOAT32);
            add_field("y", offsetof(sensor_base::LidarDataPointLayout, y), sensor_msgs::msg::PointField::FLOAT32);
            add_field("z", offsetof(sensor_base::LidarDataPointLayout, z), sensor_msgs::msg::PointField::FLOAT32);
            add_field("intensity", offsetof(sensor_base::LidarDataPointLayout, intensity), sensor_msgs::msg::PointField::FLOAT32); // 或者 UINT8
            add_field("tag", offsetof(sensor_base::LidarDataPointLayout, tag), sensor_msgs::msg::PointField::UINT8);
            add_field("line", offsetof(sensor_base::LidarDataPointLayout, line), sensor_msgs::msg::PointField::UINT8);
            
            // 如果你的 offset_time 是 uint64_t，在 ROS 里通常不直接映射那么大的自定义整型。
            // 但如果强行映射，这里可能得填 FLOAT64 或自定义解析。如果是相对时间偏移，可以用 FLOAT32/FLOAT64。
            // add_field("offset_time", offsetof(sensor_base::LidarDataPointLayout, offset_time), ...);
        }

        // 🌟 护城河 2：一步到位分配底层内存（对象池重用时，如果容量够，resize 是不耗时的）
        msg.data.resize(points_num * sizeof(sensor_base::LidarDataPointLayout));
        
        // 🌟 绝杀优化：获取字节流底层指针，直接强转为你熟悉的结构体指针！
        auto* dest_ptr = reinterpret_cast<sensor_base::LidarDataPointLayout*>(msg.data.data());

        // 像操作普通数组一样，直接在 ROS msg 的内存里进行赋值和修正，0 次额外内存分配！
        for(uint32_t i = 0; i < points_num; ++i) {
            if(do_transform) {
                Eigen::Vector3f p(points[i].x, points[i].y, points[i].z);
                p = transform * p;
                dest_ptr[i].x = p.x();
                dest_ptr[i].y = p.y();
                dest_ptr[i].z = p.z();
            } else {
                dest_ptr[i].x = points[i].x;
                dest_ptr[i].y = points[i].y;
                dest_ptr[i].z = points[i].z;
            }
            dest_ptr[i].intensity = points[i].intensity;
            dest_ptr[i].tag = points[i].tag;
            dest_ptr[i].line = points[i].line;
            // 在内存里就地修正时间戳
            dest_ptr[i].offset_time = points[i].offset_time - task.timestamp_nanos;
        }
    }


} // namespace lidar_controller


// struct Params {
//         std::vector<std::string> lidar_list = {"mid360_front/lidar"};
//         struct LidarConfig {
//             struct MapLidarList {
//                 bool enable_dynamic_extrinsics = false;
//             };
//             std::map<std::string, MapLidarList> lidar_list_map;
//         } lidar_config;
//         // for detecting if the parameter struct has been updated
//         rclcpp::Time __stamp;
//     };

