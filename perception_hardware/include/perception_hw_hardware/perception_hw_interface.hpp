#ifndef _PERCEPTION_HARDWARE_PERCEPTION_HW_INTERFACE_HPP_
#define _PERCEPTION_HARDWARE_PERCEPTION_HW_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "perception_hw_hardware/snesor_base_types.hpp"


namespace perception_hw_hardware {

class PerceptionHwInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(PerceptionHwInterface)

    hardware_interface::CallbackReturn on_init(
        const hardware_interface::HardwareInfo & info) override;
    
    hardware_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
    
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    // std::unique_ptr<sensor_base::UsbCameraSensor> usb_camera_sensor_;
    
};


} // namespace perception_hw_hardware

#endif // _PERCEPTION_HARDWARE_PERCEPTION_HW_INTERFACE_HPP_
