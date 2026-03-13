#include "mid360_lidar_base/mid360_lidar_sensor.hpp"
#include "sensor_base/sensor_base.hpp"

namespace sensor_base {
    
Mid360LidarSensor::Mid360LidarSensor(const std::string & name)
    : SensorBase(name) 
{
    name_ = name;
}

Mid360LidarSensor::~Mid360LidarSensor() {close_device();}

bool Mid360LidarSensor::init(const Mid360LidarConfig & config)
{
    config_ = config;
    data_1_ = Mid360LidarData();
    // data_1_.pose.header.update_count = 0;
    // data_1_.gyro.header.update_count = 0;
    // data_1_.accel.header.update_count = 0;
    // data_1_.fisheye0.header.update_count = 0;
    // data_1_.fisheye1.header.update_count = 0;
    data_2_ = Mid360LidarData();
    // data_2_.pose.header.update_count = 0;
    // data_2_.gyro.header.update_count = 0;
    // data_2_.accel.header.update_count = 0;
    // data_2_.fisheye0.header.update_count = 0;
    // data_2_.fisheye1.header.update_count = 0;
    return true;
}

bool Mid360LidarSensor::open_device()
{

    return true;
}

bool Mid360LidarSensor::close_device()
{
    return true;
}

void Mid360LidarSensor::main_loop()
{
    // todo
}

bool Mid360LidarSensor::update_buffer2()
{
    return true;
}



} // namespace sensor_base
