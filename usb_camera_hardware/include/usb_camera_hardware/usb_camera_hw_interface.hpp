#ifndef _USB_CAMERA_HARDWARE_USB_CAMERA_HW_INTERFACE_HPP_
#define _USB_CAMERA_HARDWARE_USB_CAMERA_HW_INTERFACE_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"

#include "usb_camera_hardware/usb_camera_sensor.hpp"

namespace usb_camera_hardware {

class UsbCameraHwInterface : public hardware_interface::SystemInterface {
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(UsbCameraHwInterface)

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
    std::unique_ptr<sensor_base::UsbCameraSensor> usb_camera_sensor_;
    
};


} // namespace usb_camera_hardware

#endif // _USB_CAMERA_HARDWARE_USB_CAMERA_HW_INTERFACE_HPP_