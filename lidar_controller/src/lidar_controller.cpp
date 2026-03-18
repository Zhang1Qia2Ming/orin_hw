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

        
        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::CallbackReturn LidarController::on_activate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = true;

        
        return controller_interface::CallbackReturn::SUCCESS;
    }
    controller_interface::InterfaceConfiguration LidarController::state_interface_configuration() const {
        controller_interface::InterfaceConfiguration config;

        config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
        
        return config;
    }

    controller_interface::InterfaceConfiguration LidarController::command_interface_configuration() const {
        return controller_interface::InterfaceConfiguration{
            controller_interface::interface_configuration_type::NONE
        };
    }

    controller_interface::CallbackReturn LidarController::on_deactivate(const rclcpp_lifecycle::State& previous_state) {
        is_running_ = false;


        return controller_interface::CallbackReturn::SUCCESS;
    }

    controller_interface::return_type LidarController::update(const rclcpp::Time& time, const rclcpp::Duration& period) {
        
        // process lidar
        

        return controller_interface::return_type::OK;
    }
   

} // namespace lidar_controller

