#ifndef _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_
#define _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_

#include "sensor_base/sensor_base.hpp"
#include "sensor_base/data_layouts.hpp"
#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp>

#include <thread>
#include <atomic>
#include <regex>
#include <memory>


namespace sensor_base {

struct Mid360LidarConfig {
    
};

struct Mid360LidarData {
    // lidar data
};

class Mid360LidarSensor : public SensorBase {
public:

    // double buffer: data_1_ and data_2_
    Mid360LidarData data_1_;
    Mid360LidarData data_2_;

    // pointer
    

    std::timed_mutex data_mutex_;

    Mid360LidarSensor(const std::string & name);
    ~Mid360LidarSensor();

    bool init() override {return true;}
    bool init(const Mid360LidarConfig & config);
    bool open_device();
    bool close_device();
    bool update_buffer2();

protected:
    void main_loop() override;
            
private:


private:
    std::string name_;

    // config
    Mid360LidarConfig config_;
    
};

} // namespace sensor_base

#endif // _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_
