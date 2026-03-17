#include "mid360_lidar_base/mid360_lidar_sensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>
#include <vector>

using namespace sensor_base;
int main(int argc, char** argv) {
    // mock config
    Mid360LidarConfig config;
    config.xfer_format = 4;
    config.multi_topic = 0;
    config.data_src = 0;
    config.publish_freq = 10.0;
    config.output_type = 0;
    config.frame_id = "lidar_frame";
    config.cmdline_input_bd_code = "livox0000000001";
    config.pcl_data_type = kLivoxLidarCartesianCoordinateHighData;
    config.lidar_ip = "192.168.1.168";
    config.lidar_cmd_data_port = 56100;
    config.lidar_publish_msg_port = 56200;
    config.lidar_point_data_port = 56300;
    config.lidar_imu_data_port = 56400;
    config.lidar_log_data_port = 56500;
    config.host_ip = "192.168.1.50";
    config.host_cmd_data_port = 56101;
    config.host_publish_msg_port = 56201;
    config.host_point_data_port = 56301;
    config.host_imu_data_port = 56401;
    config.host_log_data_port = 56501;
    config.extrinsic_parameter[0] = 0.0;
    config.extrinsic_parameter[1] = 0.0;
    config.extrinsic_parameter[2] = 0.0;
    config.extrinsic_parameter[3] = 0.0;
    config.extrinsic_parameter[4] = 0.0;
    config.extrinsic_parameter[5] = 0.0;
    std::string name = "mid360_lidar_front";
    
    Mid360LidarSensor mid360_lidar_sensor(name);
    mid360_lidar_sensor.init(config);
    std::cout << "open_device" << std::endl;
    mid360_lidar_sensor.open_device();

    while(true) {
        
    }

}


using namespace std::chrono;



// struct Mid360LidarConfig {
//     int xfer_format;
//     int multi_topic;
//     int data_src;
//     double publish_freq;
//     int output_type;
//     std::string frame_id;
//     std::string cmdline_input_bd_code;
//     LivoxLidarPointDataType pcl_data_type;

//     // lidar side
//     std::string lidar_ip;
//     int lidar_cmd_data_port;
//     int lidar_publish_msg_port;
//     int lidar_point_data_port;
//     int lidar_imu_data_port;
//     int lidar_log_data_port;

//     // host side
//     std::string host_ip;
//     int host_cmd_data_port;
//     int host_publish_msg_port;
//     int host_point_data_port;
//     int host_imu_data_port;
//     int host_log_data_port;

//     // extrinsic
//     // roll pitch yaw x y z
//     double extrinsic_parameter[6];
//     bool is_extrinsic_set = false;
//     float TranslationVector[3] = {0.0f, 0.0f, 0.0f}; /**< x, y, z translation, unit: mm. */
//     float RotationMatrix[3][3] = 
//         {
//             {0.0f, 0.0f, 0.0f}, 
//             {0.0f, 0.0f, 0.0f}, 
//             {0.0f, 0.0f, 0.0f}};
// };