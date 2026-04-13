#include "imu_controller/imu_controller.hpp"
#include "sensor_base/data_layouts.hpp"


namespace imu_controller {
    ImuController::ImuController() : controller_interface::ControllerInterface() {}

    controller_interface::CallbackReturn ImuController::on_init() {
        try {
            param_listener_ = std::make_shared<ParamListener>(get_node());
        } catch (const std::exception& e) {
            RCLCPP_ERROR(get_node()->get_logger(), "Exception in on_init: %s", e.what());
            return controller_interface::CallbackReturn::ERROR;
        }
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImuController::on_configure(const rclcpp_lifecycle::State& previous_state) {
        // get params
        params_ = param_listener_->get_params();

        // device_name -> imu_stream_context
        std::map<std::string, std::shared_ptr<ImuStreamContext>> imu_interface_map;
        
        // classify imu interfaces
        // imu = accel + gyro
        for (const auto& interface_name : params_.imu_list) {
            size_t pos = interface_name.find_last_of('/');
            if (pos == std::string::npos) {
                RCLCPP_WARN(get_node()->get_logger(),
                 "Invalid interface format: %s", interface_name.c_str());
                continue;
            }
            std::string device_name = interface_name.substr(0, pos);
            std::string suffix = interface_name.substr(pos + 1);

            if(imu_interface_map.find(device_name) == imu_interface_map.end()) {
                auto ctx = std::make_shared<ImuStreamContext>();
                ctx->device_name = device_name;
                ctx->topic_name = device_name + "/imu";
                // rclcpp::QoS imu_qos = rclcpp::SensorDataQoS();
                // imu_qos.reliable();
                ctx->imu_pub_ = get_node()->create_publisher<sensor_msgs::msg::Imu>(
                    ctx->topic_name, 256);
                
                imu_interface_map[device_name] = ctx;
            }

            auto ctx = imu_interface_map[device_name];
            if(suffix == "imu") {
                ctx->interface_name = interface_name;
                ctx->frame_id = "livox_frame";
            } else if(suffix == "gyro") {
                ctx->gyro_interface_name = interface_name;
                ctx->frame_id = device_name + "_frame";
            } else if (suffix == "accel") {
                ctx->accel_interface_name = interface_name;
                ctx->frame_id = device_name + "_frame";
            } else {
                // unknown suffix
            }
            // RCLCPP_INFO(get_node()->get_logger(),
            //  "Classified imu interface: %s, suffix: %s", interface_name.c_str(), suffix.c_str());
        }
        RCLCPP_INFO(get_node()->get_logger(), "Classified %zu imu interfaces.", imu_interface_map.size());

        for(auto& pair : imu_interface_map) {
            const std::string& device_name = pair.first;
            auto ctx = pair.second;

            // check only two situations: 1.imu ; 2.accel + gyro
            if(!ctx->interface_name.empty() || 
            (!ctx->accel_interface_name.empty() && !ctx->gyro_interface_name.empty())) {
                imu_stream_contexts_.push_back(ctx);
            } else {
                // error
            }
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImuController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        for(auto& ctx : imu_stream_contexts_) {
            ctx->last_update_count = 0;
            ctx->thread_ = std::thread(&ImuController::worker_thread, this, ctx);
        }

        RCLCPP_INFO(get_node()->get_logger(), "Imu stream activated ! %zu worker threads started.", imu_stream_contexts_.size());
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::InterfaceConfiguration ImuController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        // int num = 0;
        for(const auto& ctx : imu_stream_contexts_) {
            // num++;
            if(!ctx->interface_name.empty()) {
                config.names.push_back(ctx->interface_name);
            } else {
                config.names.push_back(ctx->accel_interface_name);
                config.names.push_back(ctx->gyro_interface_name);
            }
        }
        // RCLCPP_INFO(get_node()->get_logger(), "Classified %d state interfaces.", num);

        // for(const auto& name : config.names) {
        //     RCLCPP_INFO(get_node()->get_logger(), "State interface: %s", name.c_str());
        // }
        return config;
    }

    controller_interface::InterfaceConfiguration ImuController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::CallbackReturn ImuController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = false;

        for(auto& ctx : imu_stream_contexts_) {
            ctx->cv_.notify_all();
            if(ctx->thread_.joinable()) {
                ctx->thread_.join();
            }

            std::queue<ImuPublishTask> empty_queue;
            {
                std::lock_guard<std::mutex> lock(ctx->mutex_);
                std::swap(ctx->queue_, empty_queue);
            }
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type ImuController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        
        size_t expected_interface_size = 0;
        for(auto& ctx : imu_stream_contexts_) {
            if(!ctx->interface_name.empty()) {
                expected_interface_size++;
            } else if(!ctx->accel_interface_name.empty() && !ctx->gyro_interface_name.empty()) {
                expected_interface_size += 2;
            }
        }
        if(state_interfaces_.size() != expected_interface_size) {
            return controller_interface::return_type::OK;
        }

        // RCLCPP_INFO(get_node()->get_logger(), "update %zu imu streams!!!", imu_stream_contexts_.size());
        for(size_t i = 0;i< imu_stream_contexts_.size();i++) {
            auto ctx = imu_stream_contexts_[i];

            if(!ctx->interface_name.empty()) {
                double imu_ptr_value = 0.0;

                bool found_imu_interface = false;

                // shield for imu
                for (const auto& interface : state_interfaces_) {
                    if (interface.get_name() == ctx->interface_name) {
                        try {
                            imu_ptr_value = interface.get_value();
                            found_imu_interface = true;
                            break; 
                        } catch (const std::exception& e) {
                            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                "[%s] HW interface error: %s", ctx->interface_name.c_str(), e.what());
                            break;
                        }
                    }
                }

                // /device_name/imu -> topic pub
                if(imu_ptr_value == 0.0) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                "HW interface error: imu_ptr_value is 0.0");
                }

                if(!found_imu_interface || std::isnan(imu_ptr_value) || imu_ptr_value == 0.0){
                    continue;
                }

                sensor_base::ImuPointerPack* imu_ptr_pack = nullptr;
                std::memcpy(&imu_ptr_pack, &imu_ptr_value, sizeof(imu_ptr_pack));

                sensor_base::GyroDataLayout* get_gyro_ptr = imu_ptr_pack->gyro_ptr;
                sensor_base::AccelDataLayout* get_accel_ptr = imu_ptr_pack->accel_ptr;

                if(get_gyro_ptr == nullptr || get_accel_ptr == nullptr) {
                    continue;
                }

                // gyro for t265 hz:200, accel for t265 hz:62.5,so we choose gyro update_count
                if(get_gyro_ptr->header.update_count > ctx->last_update_count) {
                    
                    //update local
                    ctx->last_update_count = get_gyro_ptr->header.update_count;

                    ImuPublishTask task;
                    task.timestamp_nanos = get_gyro_ptr->header.timestamp_nanos;
                    task.update_count = get_gyro_ptr->header.update_count;
                    std::memcpy(&task.gyro, &get_gyro_ptr->gyro, sizeof(task.gyro));
                    std::memcpy(&task.accel, &get_accel_ptr->accel, sizeof(task.accel));        

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


            } else {
                double gyro_ptr_value = 0.0;
                double accel_ptr_value = 0.0;

                bool found_gyro_interface = false;
                bool found_accel_interface = false;

                // shield for gyro
                for (const auto& interface : state_interfaces_) {
                    if (interface.get_name() == ctx->gyro_interface_name) {
                        try {
                            gyro_ptr_value = interface.get_value();
                            found_gyro_interface = true;
                            break; 
                        } catch (const std::exception& e) {
                            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                "[%s] HW interface error: %s", ctx->gyro_interface_name.c_str(), e.what());
                            break;
                        }
                    }
                }

                // shield for accel
                for (const auto& interface : state_interfaces_) {
                    if (interface.get_name() == ctx->accel_interface_name) {
                        try {
                            accel_ptr_value = interface.get_value();
                            found_accel_interface = true;
                            break; 
                        } catch (const std::exception& e) {
                            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                "[%s] HW interface error: %s", ctx->accel_interface_name.c_str(), e.what());
                            break;
                        }
                    }
                }

                if(gyro_ptr_value == 0.0 || accel_ptr_value == 0.0) {
                    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                                "HW interface error: gyro_ptr_value or accel_ptr_value is 0.0");
                }

                if(!found_gyro_interface || !found_accel_interface || std::isnan(gyro_ptr_value) || std::isnan(accel_ptr_value) || gyro_ptr_value == 0.0 || accel_ptr_value == 0.0) {
                    continue;
                }

                // /device_name/gyro + /device_name/accel -> topic pub
                sensor_base::GyroDataLayout* get_gyro_ptr = nullptr;
                sensor_base::AccelDataLayout* get_accel_ptr = nullptr;
                
                std::memcpy(&get_gyro_ptr, &gyro_ptr_value, sizeof(get_gyro_ptr));
                std::memcpy(&get_accel_ptr, &accel_ptr_value, sizeof(get_accel_ptr));

                if(get_gyro_ptr == nullptr || get_accel_ptr == nullptr) {
                    continue;
                }

                // gyro for t265 hz:200, accel for t265 hz:62.5,so we choose gyro update_count
                if(get_gyro_ptr->header.update_count > ctx->last_update_count) {
                    
                    //update local
                    ctx->last_update_count = get_gyro_ptr->header.update_count;

                    ImuPublishTask task;
                    task.timestamp_nanos = get_gyro_ptr->header.timestamp_nanos;
                    task.update_count = get_gyro_ptr->header.update_count;
                    std::memcpy(&task.gyro, &get_gyro_ptr->gyro, sizeof(task.gyro));
                    std::memcpy(&task.accel, &get_accel_ptr->accel, sizeof(task.accel));        

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
            
        }

        return controller_interface::return_type::OK;
    }

    void ImuController::worker_thread(std::shared_ptr<ImuStreamContext> ctx) {
        RCLCPP_INFO(get_node()->get_logger(), "Imu stream %s worker thread started", ctx->interface_name.c_str());
        std::string t_name = ctx->interface_name;
        if(t_name.length() > 15) {
            t_name = t_name.substr(0, 15);
        }
        int rc = pthread_setname_np(pthread_self(), t_name.c_str());
        if(rc != 0) {
            RCLCPP_WARN(get_node()->get_logger(),"fail to set thread name for imu stream %s", ctx->interface_name.c_str());
        }
        while(is_running_) {
            
            ImuPublishTask task;
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
            header.frame_id = ctx->frame_id;

            auto imu_msg = std::make_shared<sensor_msgs::msg::Imu>();
            imu_msg->header = header;

            imu_msg->orientation.x = 0.0;
            imu_msg->orientation.y = 0.0;
            imu_msg->orientation.z = 0.0;
            imu_msg->orientation.w = 1.0;
            
            // imu_msg->orientation_covariance[0] = -1.0;

            imu_msg->linear_acceleration.x = task.accel[0];
            imu_msg->linear_acceleration.y = task.accel[1];
            imu_msg->linear_acceleration.z = task.accel[2];
            imu_msg->angular_velocity.x = task.gyro[0];
            imu_msg->angular_velocity.y = task.gyro[1];
            imu_msg->angular_velocity.z = task.gyro[2];

            // imu_msg->linear_acceleration_covariance[0] = 0.01;
            // imu_msg->linear_acceleration_covariance[4] = 0.01;
            // imu_msg->linear_acceleration_covariance[8] = 0.01;
            // imu_msg->angular_velocity_covariance[0] = 0.01;
            // imu_msg->angular_velocity_covariance[4] = 0.01;
            // imu_msg->angular_velocity_covariance[8] = 0.01;

            try {
                // publish imu message
                ctx->imu_pub_->publish(std::move(*imu_msg));
            } catch(const std::exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "Imu publish exception: %s", e.what());
            }
        }
    }
} // namespace imu_controller

