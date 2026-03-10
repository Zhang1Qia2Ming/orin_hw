#ifndef IMU_CONTROLLER__IMU_CONTROLLER_HPP_
#define IMU_CONTROLLER__IMU_CONTROLLER_HPP_

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
#include <rclcpp/rclcpp.hpp>
#include <imu_controller/imu_controller_parameters.hpp>
#include <sensor_base/data_layouts.hpp>

namespace imu_controller {


struct ImuPublishTask {
    uint64_t timestamp_nanos{0};
    uint64_t update_count;
    double gyro[3];
    double accel[3];
};

struct ImuStreamContext {
    std::string device_name;
    std::string interface_name;         // if interface_name is imu, use only interface_name
    std::string gyro_interface_name;    // otherwise, use gyro_interface_name and accel_interface_name
    std::string accel_interface_name;

    std::string topic_name;             // but the topic_name is only /device_name/imu

    uint64_t last_update_count{0};

    std::queue<ImuPublishTask> queue_;

    std::mutex mutex_;
    std::condition_variable cv_;
    std::thread thread_;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ImuController : public controller_interface::ControllerInterface {

    private:
        
    public:
        ImuController();

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
        void worker_thread(std::shared_ptr<ImuStreamContext> imu_stream_context);

        std::vector<std::shared_ptr<ImuStreamContext>> imu_stream_contexts_;
        
        // params getter
        std::shared_ptr<imu_controller::ParamListener> param_listener_;
        imu_controller::Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace imu_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    imu_controller::ImuController, controller_interface::ControllerInterface)

#endif // IMU_CONTROLLER__IMU_CONTROLLER_HPP_
