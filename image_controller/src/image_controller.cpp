#include "image_controller/image_controller.hpp"
#include "sensor_base/data_layouts.hpp"

#include "pthread.h"

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

            if(params_.image_config.image_list_map.count(interface_name) != 0) {
                image_stream_context->config = params_.image_config.image_list_map.at(interface_name);
            } else {
                // error
            }

            if(image_stream_context->config.publish_raw) {
                image_stream_context->image_raw_pub_ = get_node()->create_publisher<sensor_msgs::msg::Image>(
                    image_stream_context->topic_name + "/raw", 10);
            }
            if(image_stream_context->config.publish_undistorted) {
                image_stream_context->image_undistorted_pub_ = get_node()->create_publisher<sensor_msgs::msg::Image>(
                    image_stream_context->topic_name + "/undistorted", 10);
            }
            if(image_stream_context->config.publish_compressed) {
                image_stream_context->compressed_pub_ = get_node()->create_publisher<sensor_msgs::msg::CompressedImage>(
                    image_stream_context->topic_name + "/compressed", 10);
            }

            image_stream_contexts_.push_back(image_stream_context);

            RCLCPP_INFO(get_node()->get_logger(), "Configured stream: %s; Raw: %s; Undistorted: %s; Compressed: %s", 
                        interface_name.c_str(), 
                        image_stream_context->config.publish_raw ? "Yes" : "No", 
                        image_stream_context->config.publish_undistorted ? "Yes" : "No",
                        image_stream_context->config.publish_compressed ? "Yes" : "No");
        }

        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn ImageController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        int processer_count = 1;
        for(auto& ctx : image_stream_contexts_) {
            ctx->last_update_count = 0;
          
            for(int i = 0;i<processer_count;i++) {
                ctx->processed_threads_.emplace_back(std::thread(&ImageController::processor_thread, this, ctx));
            }

            ctx->publish_thread_ = std::thread(&ImageController::publisher_thread, this, ctx);
        }

        RCLCPP_INFO(get_node()->get_logger(), "Image stream activated ! %zu publisher threads started, %zu processed threads started.", image_stream_contexts_.size(), image_stream_contexts_.size()*processer_count);
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
            ctx->processed_cv_.notify_all();

            // join processed threads
            for(auto& thread : ctx->processed_threads_) {
                if(thread.joinable()) {
                    thread.join();
                }
            }
            ctx->processed_threads_.clear();

            // join publisher thread
            if(ctx->publish_thread_.joinable()) {
                ctx->publish_thread_.join();
            }
            
            // clear queues safely
            {
                std::lock_guard<std::mutex> lock(ctx->mutex_);
                ctx->queue_.clear();
            }
            {
                std::lock_guard<std::mutex> lock(ctx->processor_mutex_);
                ctx->processed_queue_raw.clear();
                ctx->processed_queue_undistorted.clear();
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
                task.image = get_image_ptr->image.clone();

                {
                    std::lock_guard<std::mutex> lock(ctx->mutex_);
                    ctx->queue_.push_back(std::move(task));

                    if(ctx->queue_.size() > 10) {
                        ctx->queue_.pop_front();
                    }                   
                }
                ctx->cv_.notify_one();
            }
        }

        return controller_interface::return_type::OK;
    }


    // todo :segment fault,maybe visit beyond boundary
    // e.m. std::vector's size is 7, capacity is 10, when you visit, error may not happen
    void ImageController::processor_thread(std::shared_ptr<ImageStreamContext> ctx) {
        // set_thread_name(ctx->interface_name + "_processor_thread");
        
        while(is_running_) {
            ImagePublishTask task;
            {
                std::unique_lock<std::mutex> lock(ctx->mutex_);

                // wait for new task or stop signal
                ctx->cv_.wait(lock, [&ctx, this] {
                    return !ctx->queue_.empty() || !is_running_;
                });

                if(!is_running_ && ctx->queue_.empty()) {
                    break;
                }
                task = std::move(ctx->queue_.front());
                ctx->queue_.pop_front();
            }

            if(task.image.empty()) {
                continue;
            }

            // lazy init maps
            if(!ctx->map_initialized) {
                init_maps(ctx, task.image.size());
            }
            cv::Mat processed_image;
            cv::remap(task.image, processed_image, ctx->map1, ctx->map2, cv::INTER_LINEAR);

            //header
            std_msgs::msg::Header header;
            header.stamp.sec = task.timestamp_nanos / 1000000000ULL;
            header.stamp.nanosec = task.timestamp_nanos % 1000000000ULL;
            header.frame_id = ctx->interface_name.substr(0, ctx->interface_name.find("/"));

            std::string encoding = (processed_image.channels() == 1) ? "mono8" : "bgr8";

            sensor_msgs::msg::Image::SharedPtr raw_img;
            sensor_msgs::msg::Image::SharedPtr undistorted_img;
            sensor_msgs::msg::CompressedImage::SharedPtr compressed_img;

            try {
                if(ctx->config.publish_raw) {
                    raw_img = cv_bridge::CvImage(header, encoding, task.image).toImageMsg();
                }
                if(ctx->config.publish_undistorted) {
                    undistorted_img = cv_bridge::CvImage(header, encoding, processed_image).toImageMsg();
                }
                if(ctx->config.publish_compressed) {

                    std::vector<uchar> buf;
                    std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 98};
                    cv::imencode(".jpg", task.image, buf, params);
        
                    // 创建压缩图像消息
                    compressed_img = std::make_shared<sensor_msgs::msg::CompressedImage>();
                    compressed_img->header.stamp = header.stamp;
                    compressed_img->header.frame_id = header.frame_id;
                    compressed_img->format = "jpeg";
                    compressed_img->data = std::move(buf);
                }
            } catch(const cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "cv_bridge exception: %s", e.what());
                continue;
            }

            // push to processor queue
            {
                std::lock_guard<std::mutex> lock(ctx->processor_mutex_);
                if(ctx->processed_queue_raw.size() > 10) {
                    ctx->processed_queue_raw.pop_front();
                }
                if(ctx->processed_queue_undistorted.size() > 10) {
                    ctx->processed_queue_undistorted.pop_front();
                }
                if(ctx->processed_queue_compressed.size() > 10) {
                    ctx->processed_queue_compressed.pop_front();
                }
                if(raw_img) {
                    ctx->processed_queue_raw.push_back(raw_img);
                }
                if(undistorted_img) {
                    ctx->processed_queue_undistorted.push_back(undistorted_img);
                }
                if(compressed_img) {
                    ctx->processed_queue_compressed.push_back(compressed_img);
                }
            }
            ctx->processed_cv_.notify_one();
        }
    }

    void ImageController::publisher_thread(std::shared_ptr<ImageStreamContext> ctx) {
        // set_thread_name(ctx->interface_name + "_publisher_thread");
        while(is_running_) {
            
            sensor_msgs::msg::Image::SharedPtr msg_raw = nullptr;
            sensor_msgs::msg::Image::SharedPtr msg_undistorted = nullptr;
            sensor_msgs::msg::CompressedImage::SharedPtr msg_compressed = nullptr;

            {
                std::unique_lock<std::mutex> lock(ctx->processor_mutex_);
                
                ctx->processed_cv_.wait(lock, [&] {
                    return !ctx->processed_queue_raw.empty() || !ctx->processed_queue_undistorted.empty() || !ctx->processed_queue_compressed.empty() || !is_running_;
                });
                if(!ctx->processed_queue_raw.empty()) {
                    msg_raw = ctx->processed_queue_raw.front();
                    ctx->processed_queue_raw.pop_front();
                }
                if(!ctx->processed_queue_undistorted.empty()) {
                    msg_undistorted = ctx->processed_queue_undistorted.front();
                    ctx->processed_queue_undistorted.pop_front();
                }
                if(!ctx->processed_queue_compressed.empty()) {
                    msg_compressed = ctx->processed_queue_compressed.front();
                    ctx->processed_queue_compressed.pop_front();
                }
            }
            try {
                if(ctx->config.publish_raw && msg_raw) {

                    // size_t payload_bytes = msg_raw->data.size();
                    // double payload_kb = payload_bytes / 1024.0;
                    // double payload_mb = payload_kb / 1024.0;
                    // RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
                    //     "Image stream %s publish payload kb: %f, payload mb: %f", 
                    //     ctx->interface_name.c_str(), payload_kb, payload_mb);

                    ctx->image_raw_pub_->publish(*msg_raw);
                }

                if(ctx->config.publish_undistorted && msg_undistorted) {
                    ctx->image_undistorted_pub_->publish(*msg_undistorted);
                }
                if(ctx->config.publish_compressed && msg_compressed) {
                    ctx->compressed_pub_->publish(*msg_compressed);
                }
            } catch(const cv_bridge::Exception& e) {
                RCLCPP_ERROR(get_node()->get_logger(), "pub img failed: %s", e.what());
            }
        }
    }

    void ImageController::init_maps(std::shared_ptr<ImageStreamContext> ctx, cv::Size image_size) {
        cv::Mat K = cv::Mat(3, 3, CV_64F, ctx->config.intrinsic_matrix.data());
        cv::Mat D = cv::Mat(ctx->config.distortion.size(), 1, CV_64F, ctx->config.distortion.data());

        cv::initUndistortRectifyMap(K, D, cv::Mat(), K, image_size, CV_32FC1, ctx->map1, ctx->map2);

        ctx->map_initialized = true;
    }

} // namespace image_controller

