#ifndef LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
#define LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_

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
#include "controller_msg/msg/custom_point.hpp"
#include "controller_msg/msg/custom_msg.hpp"



#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <lidar_controller/lidar_controller_parameters.hpp>

namespace lidar_controller {


struct LidarPublishTask {
    
};

struct LidarStreamContext {
    
};

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class LidarController : public controller_interface::ControllerInterface {

    private:
        
    public:
        LidarController();

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
        

        std::vector<std::shared_ptr<LidarStreamContext>> lidar_stream_contexts_;
        
        // params getter
        std::shared_ptr<lidar_controller::ParamListener> param_listener_;
        lidar_controller::Params params_;

        // about thread
        std::vector<std::thread> threads_;
        std::atomic<bool> is_running_{false};
};

} // namespace image_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    lidar_controller::LidarController, controller_interface::ControllerInterface)

#endif // LIDAR_CONTROLLER__LIDAR_CONTROLLER_HPP_
