#include "mid360_lidar_hardware/mid360_lidar_hw_interface.hpp"
#include "mid360_lidar_hardware/sensor_base_types.hpp"
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

namespace mid360_lidar_hardware {

hardware_interface::CallbackReturn Mid360LidarHwInterface::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    mid360_lidar_sensor_ = std::make_unique<sensor_base::Mid360LidarSensor>(info.name);
    
    sensor_base::Mid360LidarConfig config;
    config.xfer_format = std::stoi(info.hardware_parameters.find("xfer_format")->second);
    config.multi_topic = std::stoi(info.hardware_parameters.find("multi_topic")->second);
    config.data_src = std::stoi(info.hardware_parameters.find("data_src")->second);
    config.publish_freq = std::stod(info.hardware_parameters.find("publish_freq")->second);
    config.output_type = std::stoi(info.hardware_parameters.find("output_type")->second);
    config.frame_id = info.hardware_parameters.find("frame_id")->second;
    config.cmdline_input_bd_code = info.hardware_parameters.find("cmdline_input_bd_code")->second;
    config.pcl_data_type = kLivoxLidarCartesianCoordinateHighData;
    
    config.extrinsic_parameter[0] = std::stod(info.hardware_parameters.find("extrinsic_parameter_roll")->second);
    config.extrinsic_parameter[1] = std::stod(info.hardware_parameters.find("extrinsic_parameter_pitch")->second);
    config.extrinsic_parameter[2] = std::stod(info.hardware_parameters.find("extrinsic_parameter_yaw")->second);
    config.extrinsic_parameter[3] = std::stod(info.hardware_parameters.find("extrinsic_parameter_x")->second);
    config.extrinsic_parameter[4] = std::stod(info.hardware_parameters.find("extrinsic_parameter_y")->second);
    config.extrinsic_parameter[5] = std::stod(info.hardware_parameters.find("extrinsic_parameter_z")->second);

    // config.serial_no = info.hardware_parameters.find("serial_no")->second;
    // config.usb_port_id = info.hardware_parameters.find("usb_port_id")->second;
    // config.device_type = info.hardware_parameters.find("device_type")->second;
    // config.wait_for_device_timeout = std::stod(info.hardware_parameters.find("wait_for_device_timeout")->second);
    // config.reconnect_timeout = std::stod(info.hardware_parameters.find("reconnect_timeout")->second);
    // config.initial_reset = (info.hardware_parameters.find("initial_reset")->second == "true")? true: false;

    if(!mid360_lidar_sensor_->init(config))
    {
        return hardware_interface::CallbackReturn::ERROR;
    }


    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mid360LidarHwInterface::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mid360LidarHwInterface::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    mid360_lidar_sensor_->open_device();
    mid360_lidar_sensor_->start_thread();
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Mid360LidarHwInterface::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    mid360_lidar_sensor_->stop_thread();
    mid360_lidar_sensor_->close_device();
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> Mid360LidarHwInterface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    state_interfaces.emplace_back(hardware_interface::StateInterface(mid360_lidar_sensor_->get_name(), "lidar", 
        &(mid360_lidar_sensor_->lidar_ptr_as_double_)));
    state_interfaces.emplace_back(hardware_interface::StateInterface(mid360_lidar_sensor_->get_name(), "imu", 
        &(mid360_lidar_sensor_->imu_ptr_as_double_))); 
    return state_interfaces; 
}

std::vector<hardware_interface::CommandInterface> Mid360LidarHwInterface::export_command_interfaces()
{
    return {};
}

hardware_interface::return_type Mid360LidarHwInterface::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    // todo: double buffer pointer
    sensor_base::LidarDataLayout * latest_front_ptr = nullptr;
    if(mid360_lidar_sensor_->PullFrontBufferPointer(&latest_front_ptr))
    {
        std::memcpy(&mid360_lidar_sensor_->lidar_ptr_as_double_, &latest_front_ptr, sizeof(double));

        // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarHwInterface"), "latest_front_ptr: %p", 
        //     *reinterpret_cast<void**>(&mid360_lidar_sensor_->lidar_ptr_as_double_));
        
    }
    mid360_lidar_sensor_->update_buffer2(); 

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type Mid360LidarHwInterface::write(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    return hardware_interface::return_type::OK;
}



} // namespace mid360_lidar_hardware


#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(mid360_lidar_hardware::Mid360LidarHwInterface, hardware_interface::SystemInterface)
