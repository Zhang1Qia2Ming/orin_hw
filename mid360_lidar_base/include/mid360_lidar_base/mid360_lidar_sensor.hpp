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
#include <mutex>
#include <condition_variable>


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
    bool is_extrinsic_set = false;
    float TranslationVector[3] = {0.0f, 0.0f, 0.0f}; /**< x, y, z translation, unit: mm. */
    float RotationMatrix[3][3] = 
        {
            {0.0f, 0.0f, 0.0f}, 
            {0.0f, 0.0f, 0.0f}, 
            {0.0f, 0.0f, 0.0f}};
};

struct Mid360LidarData {
    // lidar data
    LidarDataLayout lidar_data;
    GyroDataLayout gyro_data;
    AccelDataLayout accel_data;
};


class Mid360LidarSensor : public SensorBase {
public:

    // using PointCloudsCallback = std::function<void(*, void *)>;
    // using ImuDataCallback = std::function<void(*, void *)>;
    using TimePoint = std::chrono::high_resolution_clock::time_point;


    // double buffer: data_1_ and data_2_
    Mid360LidarData data_1_;
    Mid360LidarData data_2_;

    Mid360LidarData* front_buffer_ptr_ = nullptr;
    Mid360LidarData* back_buffer_ptr_ = nullptr;
    std::mutex buffer_swap_mutex_;
    std::condition_variable buffer_ready_cv_;
    bool is_new_frame_ready_ = false;

    double lidar_ptr_as_double_ = 0.0;

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
    // void SetPointCloudsCallback(PointCloudsCallback callback, void *user_data);
    // void SetImuDataCallback(ImuDataCallback callback, void *user_data);

    static void onLivoxLidarPointCloudCallback( uint32_t handle, 
                                                const uint8_t dev_type,
                                                LivoxLidarEthernetPacket *data, 
                                                void *client_data);

    void enqueueRawPacket(  uint32_t handle, 
                            const uint8_t dev_type, 
                            LivoxLidarEthernetPacket *data);
    
    void RawDataProcess();
    void PointCloudProcess(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer);
    void LivoxLidarPointCloudProcess(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer);
    void ProcessCartesianHighPoint(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer);
    void ProcessCartesianLowPoint(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer);
    void ProcessSphericalPoint(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer);

    void SetWriteData1(bool is_write);
    uint32_t GetLidarPointCloudsSize(Mid360LidarData& lidar_data_block);
    bool PullFrontBufferPointer(Mid360LidarData** out_front_ptr);

protected:
    void main_loop() override;
    void set_extrinsic_parameter();
            
private:


private:
    std::string name_;
    // connect state: 
    LidarConnectState connect_state_ = kConnectStateOff;
    std::thread process_thread_;
    std::atomic<bool> is_running_{false};
    std::atomic<bool> is_write_data_1_{false};
    TimePoint last_pub_time_;

    uint64_t publish_interval_ = 100000000; //100 ms
    uint64_t publish_interval_tolerance_ = 100000000; //100 ms
    uint64_t publish_interval_ms_ = 100; //100 ms
    // PointCloudsCallback point_clouds_callback_;
    // void *point_clouds_user_data_ = nullptr;

    // ImuDataCallback imu_data_callback_;
    // void *imu_data_user_data_ = nullptr;

    // config
    Mid360LidarConfig config_;

    LidarExtParameter lidar_ext_param_;

    std::deque<std::vector<RawPacket>> raw_packet_queue_;
    std::condition_variable packet_condition_;
    std::mutex packet_mutex_;
    std::mutex points_clouds_mutex_;
    std::vector<LidarDataPointLayout> points_clouds_;
    
};


} // namespace sensor_base

#endif // _MID360_LIDAR_BASE_MID360_LIDAR_SENSOR_HPP_
