#ifndef _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_
#define _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_

#include "sensor_base/sensor_base.hpp"
#include "sensor_base/data_layouts.hpp"
#include "rclcpp/rclcpp.hpp"
#include "mid360_lidar_base/comm.hpp"
#include "livox_lidar_api.h"
#include "livox_lidar_def.h"

#include <thread>
#include <atomic>
#include <memory>
#include <string>


namespace sensor_base {

/** Lidar connect state */
typedef enum {
    kConnectStateOff = 0,
    kConnectStateOn = 1,
    kConnectStateConfig = 2,
    kConnectStateSampling = 3,
} LidarConnectState;


struct Mid360LidarConfig {
    int xfer_format;
    int multi_topic;
    int data_src;
    double publish_freq;
    int output_type;
    std::string frame_id;
    std::string cmdline_input_bd_code;
    LivoxLidarPointDataType pcl_data_type;

    // lidar side
    std::string lidar_ip;
    int lidar_cmd_data_port;
    int lidar_publish_msg_port;
    int lidar_point_data_port;
    int lidar_imu_data_port;
    int lidar_log_data_port;

    // host side
    std::string host_ip;
    int host_cmd_data_port;
    int host_publish_msg_port;
    int host_point_data_port;
    int host_imu_data_port;
    int host_log_data_port;

    // extrinsic
    // roll pitch yaw x y z
    double extrinsic_parameter[6];
};

struct Mid360LidarData {
    // lidar data
    LidarDataLayout data;
};


class Mid360LidarSensor : public SensorBase {
public:

    // double buffer: data_1_ and data_2_
    Mid360LidarData data_1_;
    Mid360LidarData data_2_;

    // pointer

    // queue
    LidarDataQueue lidar_data_queue_;
    LidarImuDataQueue imu_data_queue_;

    std::timed_mutex data_mutex_;

    Mid360LidarSensor(const std::string & name);
    ~Mid360LidarSensor();

    bool init() override {return true;}
    bool init(const Mid360LidarConfig & config);
    bool open_device();
    bool close_device();
    bool update_buffer2();
    void point_cloud_poll_thread();
    void imu_poll_thread();

protected:
    void main_loop() override;
            
private:


private:
    std::string name_;
    // connect state: 
    LidarConnectState connect_state_ = kConnectStateOff;

    // config
    Mid360LidarConfig config_;

    LidarExtParameter lidar_ext_param_;
    
};


} // namespace sensor_base

#endif // _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_
