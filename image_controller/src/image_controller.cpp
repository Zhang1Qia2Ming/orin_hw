#include "image_controller/image_controller.hpp"
#include "sensor_base/data_layouts.hpp"

namespace image_controller {
    ImageController::ImageController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn ImageController::on_init() {
        try {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImageController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // get params
        params_ = param_listener_->get_params();

        for (const auto& interface_name : params_.image_list) {
            // add image stream context
            auto image_stream_context = std::make_shared<ImageStreamContext>();
            image_stream_context->interface_name = interface_name;
            image_stream_context->topic_name = interface_name;

            image_stream_context->image_pub_ = get_node()->create_publisher<sensor_msgs::msg::Image>(
                image_stream_context->topic_name, 10);
            
            image_stream_context->queue_ = std::queue<ImagePublishTask>();

            image_stream_contexts_.push_back(image_stream_context);

            RCLCPP_INFO(get_node()->get_logger(), "Configured stream: %s", interface_name.c_str());
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImageController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        for(auto& ctx : image_stream_contexts_) {
            ctx->last_update_count = 0;
            ctx->thread_ = std::thread(&ImageController::worker_thread, this, ctx);
        }

        RCLCPP_INFO(get_node()->get_logger(), "Image stream activated ! %zu worker threads started.", image_stream_contexts_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ImageController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        for(const auto& image_stream_context : image_stream_contexts_) {
            config.names.push_back(image_stream_context->interface_name);
        }
        return config;
    }

    controller_interface::InterfaceConfiguration ImageController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::CallbackReturn ImageController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = false;

        for(auto& ctx : image_stream_contexts_) {
            ctx->cv_.notify_all();
            if(ctx->thread_.joinable()) {
                ctx->thread_.join();
            }

            std::queue<ImagePublishTask> empty_queue;
            {
                std::lock_guard<std::mutex> lock(ctx->mutex_);
                std::swap(ctx->queue_, empty_queue);
            }
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ImageController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        
        if(state_interfaces_.size() != image_stream_contexts_.size()) {
            return controller_interface::return_type::OK;
        }

        for(size_t i = 0;i<image_stream_contexts_.size();i++) {
            auto ctx = image_stream_contexts_[i];

            // double ptr_value = state_interfaces_[i].get_value();

            double ptr_value = 0.0;
            bool found_interface = false;

            // 🌟 1. 精准匹配：遍历底座给的接口，靠名字对暗号！绝对不错乱！
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

            sensor_base::ImageDataLayout* get_image_ptr = nullptr;
            std::memcpy(&get_image_ptr, &ptr_value, sizeof(get_image_ptr));

            if(get_image_ptr == nullptr) {
                continue;
            }

            if(get_image_ptr->header.update_count > ctx->last_update_count) {
                
                //update local
                ctx->last_update_count = get_image_ptr->header.update_count;

                ImagePublishTask task;
                task.timestamp_nanos = get_image_ptr->header.timestamp_nanos;
                task.update_count = get_image_ptr->header.update_count;
                task.image = get_image_ptr->image;

                {
                    std::lock_guard<std::mutex> lock(ctx->mutex_);
                    ctx->queue_.push(task);

                    if(ctx->queue_.size() > 10) {
                        ctx->queue_.pop();
                    }                   
                }
                ctx->cv_.notify_one();
            }
        }

        return controller_interface::return_type::OK;
    }

    void ImageController::worker_thread(std::shared_ptr<ImageStreamContext> ctx) {
        while(is_running_) {
            
            ImagePublishTask task;
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

            if(task.image.empty()) {
                continue;
            }

            std_msgs::msg::Header header;
            header.stamp.sec = task.timestamp_nanos / 1000000000ULL;
            header.stamp.nanosec = task.timestamp_nanos % 1000000000ULL;

            header.frame_id = ctx->interface_name.substr(0, ctx->interface_name.find("/"));

            std::string encoding;
            int channels = task.image.channels();
            if(channels == 1) {
                encoding = "mono8";
            } else {
                encoding = "bgr8";
            }
            try {
                auto msg = cv_bridge::CvImage(header, encoding, task.image).toImageMsg();
                ctx->image_pub_->publish(*msg);
            } catch(const cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "cv_bridge exception: %s", e.what());
            }
        }
    }
} // namespace image_controller

