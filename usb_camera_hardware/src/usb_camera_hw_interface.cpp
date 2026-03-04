#include "usb_camera_hardware/usb_camera_hw_interface.hpp"
#include "usb_camera_hardware/usb_camera_sensor.hpp"

namespace usb_camera_hardware {

hardware_interface::CallbackReturn UsbCameraHwInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    usb_camera_sensor_ = std::make_unique<sensor_base::UsbCameraSensor>(info.name);

    sensor_base::UsbCameraConfig config;
    config.video_device = info.hardware_parameters.find("video_device")->second;
    config.image_width = std::stoi(info.hardware_parameters.find("image_width")->second);
    config.image_height = std::stoi(info.hardware_parameters.find("image_height")->second);
    config.framerate = std::stod(info.hardware_parameters.find("framerate")->second);
    config.pixel_format = info.hardware_parameters.find("pixel_format")->second;
    config.camera_frame_id = info.hardware_parameters.find("camera_frame_id")->second;
    config.output_mode = info.hardware_parameters.find("output_mode")->second;

    if (!usb_camera_sensor_->init(config))
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

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
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type UsbCameraHwInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}



} // namespace usb_camera_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(usb_camera_hardware::UsbCameraHwInterface, hardware_interface::SystemInterface)
