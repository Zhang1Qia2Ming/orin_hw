#include "t265_camera_hardware/t265_camera_hw_interface.hpp"
#include "t265_camera_hardware/sensor_base_types.hpp"

namespace t265_camera_hardware {

hardware_interface::CallbackReturn T265CameraHwInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    t265_camera_sensor_ = std::make_unique<sensor_base::T265CameraSensor>(info.name);
    
    sensor_base::T265CameraConfig config;
    config.serial_no = info.hardware_parameters.find("serial_no")->second;
    config.usb_port_id = info.hardware_parameters.find("usb_port_id")->second;
    config.device_type = info.hardware_parameters.find("device_type")->second;
    config.wait_for_device_timeout = std::stod(info.hardware_parameters.find("wait_for_device_timeout")->second);
    config.reconnect_timeout = std::stod(info.hardware_parameters.find("reconnect_timeout")->second);
    config.initial_reset = (info.hardware_parameters.find("initial_reset")->second == "true")? true: false;

    if(!t265_camera_sensor_->init(config))
    {
        return hardware_interface::CallbackReturn::ERROR;
    }


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn T265CameraHwInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn T265CameraHwInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    t265_camera_sensor_->open_device();
    t265_camera_sensor_->start_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn T265CameraHwInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    t265_camera_sensor_->stop_thread();
    t265_camera_sensor_->close_device();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> T265CameraHwInterface::export_state_interfaces()
{
    // export pose, gyro, accel, image
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(t265_camera_sensor_->get_name(), "pose", 
        reinterpret_cast<double*>(&(t265_camera_sensor_->pose_ptr_))));
    state_interfaces.emplace_back(hardware_interface::StateInterface(t265_camera_sensor_->get_name(), "gyro", 
        reinterpret_cast<double*>(&(t265_camera_sensor_->gyro_ptr_))));
    state_interfaces.emplace_back(hardware_interface::StateInterface(t265_camera_sensor_->get_name(), "accel", 
        reinterpret_cast<double*>(&(t265_camera_sensor_->accel_ptr_))));
    state_interfaces.emplace_back(hardware_interface::StateInterface(t265_camera_sensor_->get_name(), "image0", 
        reinterpret_cast<double*>(&(t265_camera_sensor_->fisheye0_ptr_))));
    state_interfaces.emplace_back(hardware_interface::StateInterface(t265_camera_sensor_->get_name(), "image1", 
        reinterpret_cast<double*>(&(t265_camera_sensor_->fisheye1_ptr_))));
    return state_interfaces; 
}

std::vector<hardware_interface::CommandInterface> T265CameraHwInterface::export_command_interfaces()
{
    return {};
}

hardware_interface::return_type T265CameraHwInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // todo: copy and add mutex
    // copy from data block1 to data block2
    t265_camera_sensor_->update_buffer2();
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type T265CameraHwInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}



} // namespace t265_camera_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(t265_camera_hardware::T265CameraHwInterface, hardware_interface::SystemInterface)
