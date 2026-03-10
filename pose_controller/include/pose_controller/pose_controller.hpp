#ifndef POSE_CONTROLLER__POSE_CONTROLLER_HPP_
#define POSE_CONTROLLER__POSE_CONTROLLER_HPP_

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
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <rclcpp/rclcpp.hpp>
#include <pose_controller/pose_controller_parameters.hpp>
#include <sensor_base/data_layouts.hpp>

namespace pose_controller {


struct PosePublishTask {
    uint64_t timestamp_nanos{0};
    uint64_t update_count;
    sensor_base::PoseDataLayout pose;
};

struct PoseStreamContext {
    std::string interface_name;
    std::string topic_name;

    uint64_t last_update_count{0};

    std::queue<PosePublishTask> queue_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread thread_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class PoseController : public controller_interface::ControllerInterface {

    private:
        
    public:
        PoseController();

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
        void worker_thread(std::shared_ptr<PoseStreamContext> pose_stream_context);

        std::vector<std::shared_ptr<PoseStreamContext>> pose_stream_contexts_;
        
        // params getter
        std::shared_ptr<pose_controller::ParamListener> param_listener_;
        pose_controller::Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace pose_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    pose_controller::PoseController, controller_interface::ControllerInterface)

#endif // POSE_CONTROLLER__POSE_CONTROLLER_HPP_
