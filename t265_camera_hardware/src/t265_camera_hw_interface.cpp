#include "perception_hardware/perception_interface.hpp"
#include "perception_hw_hardware/snesor_base_types.hpp"

namespace perception_hw_hardware {

hardware_interface::CallbackReturn PerceptionHwInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // if (!usb_camera_sensor_->init(config))
    // {
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UsbCameraHwInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UsbCameraHwInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    usb_camera_sensor_->open_device();
    usb_camera_sensor_->start_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn UsbCameraHwInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    usb_camera_sensor_->stop_thread();
    usb_camera_sensor_->close_device();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> UsbCameraHwInterface::export_state_interfaces()
{
    return {};
}

std::vector<hardware_interface::CommandInterface> UsbCameraHwInterface::export_command_interfaces()
{
    return {};
}

hardware_interface::return_type UsbCameraHwInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // todo: copy and add mutex
    // copy from data block1 to data block2
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UsbCameraHwInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}



} // namespace perception_hw_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(perception_hw_hardware::PerceptionHwInterface, hardware_interface::SystemInterface)
