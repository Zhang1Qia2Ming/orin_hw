#ifndef IMAGE_CONTROLLER__IMAGE_CONTROLLER_HPP_
#define IMAGE_CONTROLLER__IMAGE_CONTROLLER_HPP_

#include <atomic>
#include <thread>
#include <map>
#include <algorithm>
#include <iterator>
#include <queue>
#include <mutex>
#include <condition_variable>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <image_controller/image_controller_parameters.hpp>

namespace image_controller {


struct ImagePublishTask {
    uint64_t timestamp_nanos{0};
    uint64_t update_count;
    cv::Mat image;
};

struct ImageStreamContext {
    std::string interface_name;
    std::string topic_name;

    uint64_t last_update_count{0};

    std::queue<ImagePublishTask> queue_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread thread_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ImageController : public controller_interface::ControllerInterface {

    private:
        
    public:
        ImageController();

        CallbackReturn on_init() override;

        controller_interface::InterfaceConfiguration state_interface_configuration() const override;

        controller_interface::InterfaceConfiguration command_interface_configuration() const override;

        CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

        CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

        controller_interface::return_type update(
            const rclcpp::Time& time, const rclcpp::Duration& period) override;
    
    public:

          
    protected:
        void worker_thread(std::shared_ptr<ImageStreamContext> image_stream_context);

        std::vector<std::shared_ptr<ImageStreamContext>> image_stream_contexts_;
        
        // params getter
        std::shared_ptr<image_controller::ParamListener> param_listener_;
        image_controller::Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace image_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    image_controller::ImageController, controller_interface::ControllerInterface)

#endif // IMAGE_CONTROLLER__IMAGE_CONTROLLER_HPP_
