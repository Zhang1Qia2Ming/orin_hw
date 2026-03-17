#include "mid360_lidar_base/mid360_lidar_sensor.hpp"
#include "sensor_base/sensor_base.hpp"
#include "mid360_lidar_base/comm.hpp"
// #include "mid360_lidar_base/pub_handler.h"

namespace sensor_base {
    
Mid360LidarSensor::Mid360LidarSensor(const std::string & name)
    : SensorBase(name) 
{
    name_ = name;
}

Mid360LidarSensor::~Mid360LidarSensor() {
    close_device();
}

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

    set_extrinsic_parameter();

    lidar_data_queue_.storage_packet = nullptr;
    lidar_data_queue_.rd_idx = 0;
    lidar_data_queue_.wr_idx = 0;
    return true;
}

bool Mid360LidarSensor::open_device()
{
    // initialize read-lidar
    int xfer_format = config_.xfer_format;
    int multi_topic = config_.multi_topic;
    int data_src = config_.data_src;
    double publish_freq = config_.publish_freq; /* Hz */
    int output_type = config_.output_type;
    std::string frame_id = config_.frame_id;
    std::string path = "/home/test/control_ws/src/orin_Hw/robot_bringup/config/MID360_config.json";
    // std::string cmdline_input_bd_code = config_.cmdline_input_bd_code;


    if (publish_freq > 100.0) {
        publish_freq = 100.0;
    } else if (publish_freq < 0.5) {
        publish_freq = 0.5;
    } else {
        publish_freq = publish_freq;
    }
    RCLCPP_INFO(rclcpp::get_logger(name_), "publish_freq: %f", publish_freq);
    // LdsLidar getInstance

    DeInitQueue(&lidar_data_queue_);
    imu_data_queue_.Clear();
    connect_state_ = kConnectStateOff;
    RCLCPP_INFO(rclcpp::get_logger(name_), "open_device");
    // Register Lds

    // Initialize Lidar according to the config file
    DisableLivoxSdkConsoleLogger();
    // std::cout << "cmdline_input_bd_code: " << cmdline_input_bd_code << std::endl;

    RCLCPP_INFO(rclcpp::get_logger(name_), "open_device1");
    // SDK initialization
    if (!LivoxLidarSdkInit(path.c_str())) {
        std::cout << "Failed to init livox lidar sdk." << std::endl;
        return false;
    }

    LivoxLidarAddPointCloudObserver(
        Mid360LidarSensor::onLivoxLidarPointCloudCallback, this);

    //
    lidar_ext_param_.lidar_type = kLivoxLidarType;
    lidar_ext_param_.handle = 0;
    if(config_.pcl_data_type == kLivoxLidarCartesianCoordinateLowData) {
        lidar_ext_param_.param.roll = config_.extrinsic_parameter[0];
        lidar_ext_param_.param.pitch = config_.extrinsic_parameter[1];
        lidar_ext_param_.param.yaw = config_.extrinsic_parameter[2];
        lidar_ext_param_.param.x = config_.extrinsic_parameter[3] / 10;
        lidar_ext_param_.param.y = config_.extrinsic_parameter[4] / 10;
        lidar_ext_param_.param.z = config_.extrinsic_parameter[5] / 10;
    } else {
        lidar_ext_param_.param.roll = config_.extrinsic_parameter[0];
        lidar_ext_param_.param.pitch = config_.extrinsic_parameter[1];
        lidar_ext_param_.param.yaw = config_.extrinsic_parameter[2];
        lidar_ext_param_.param.x = config_.extrinsic_parameter[3];
        lidar_ext_param_.param.y = config_.extrinsic_parameter[4];
        lidar_ext_param_.param.z = config_.extrinsic_parameter[5];
    }
    // pub_handler().AddLidarsExtParam(lidar_ext_param_);

    return true;
}

bool Mid360LidarSensor::close_device()
{
    // LivoxLidarRemovePointCloudObserver();
    return true;
}

void Mid360LidarSensor::main_loop()
{
    // todo
}

bool Mid360LidarSensor::update_buffer2()
{
    // todo copy data_1_ to data_2_
    return true;
}


void Mid360LidarSensor::set_extrinsic_parameter()
{
    if(config_.is_extrinsic_set) {
        return;
    }
    config_.TranslationVector[0] = config_.extrinsic_parameter[3];
    config_.TranslationVector[1] = config_.extrinsic_parameter[4];
    config_.TranslationVector[2] = config_.extrinsic_parameter[5];

    double cos_roll = cos(config_.extrinsic_parameter[0] * M_PI / 180.0);
    double cos_pitch = cos(config_.extrinsic_parameter[1] * M_PI / 180.0);
    double cos_yaw = cos(config_.extrinsic_parameter[2] * M_PI / 180.0);
    double sin_roll = sin(config_.extrinsic_parameter[0] * M_PI / 180.0);
    double sin_pitch = sin(config_.extrinsic_parameter[1] * M_PI / 180.0);
    double sin_yaw = sin(config_.extrinsic_parameter[2] * M_PI / 180.0);

    config_.RotationMatrix[0][0] = cos_pitch * cos_yaw;
    config_.RotationMatrix[0][1] = sin_roll * cos_pitch * cos_yaw - cos_roll * sin_yaw;
    config_.RotationMatrix[0][2] = cos_roll * sin_pitch * cos_yaw + sin_roll * sin_yaw;

    config_.RotationMatrix[1][0] = cos_pitch * sin_yaw;
    config_.RotationMatrix[1][1] = sin_roll * sin_pitch * sin_yaw + cos_roll * cos_yaw;
    config_.RotationMatrix[1][2] = cos_roll * sin_pitch * sin_yaw - sin_roll * cos_yaw;

    config_.RotationMatrix[2][0] = -sin_pitch;
    config_.RotationMatrix[2][1] = sin_roll * cos_pitch;
    config_.RotationMatrix[2][2] = cos_roll * cos_pitch;

    config_.is_extrinsic_set = true;
    return;
}


void Mid360LidarSensor::point_cloud_poll_thread(){

}

void Mid360LidarSensor::imu_poll_thread(){

}

void Mid360LidarSensor::onLivoxLidarPointCloudCallback( uint32_t handle, 
                                                        const uint8_t dev_type,
                                                        LivoxLidarEthernetPacket *data, 
                                                        void *client_data)
{
    Mid360LidarSensor *self = (Mid360LidarSensor *)client_data;
    if(self) {
        self->enqueueRawPacket(handle, dev_type, data);
    }
}


void Mid360LidarSensor::enqueueRawPacket(  uint32_t handle, 
                                            const uint8_t dev_type, 
                                            LivoxLidarEthernetPacket *data)
{
    // todo
    uint64_t ts = 0;
    std::memcpy(&ts, data->timestamp, sizeof(data->timestamp));

    if(data->data_type == kLivoxLidarImuData) {
        // std::cout << "imu data" << std::endl;
        

        // ========== compute imu delta time ==========
        static uint64_t last_imu_ts = 0;
        static uint32_t imu_count = 0;

        imu_count++;
        if(last_imu_ts == 0) {
            last_imu_ts = ts;
        } else {
            uint64_t dt_ns = ts - last_imu_ts;
            if(dt_ns >= 1000000000ULL) {
                double dt_sec = static_cast<double>(dt_ns) / 1e9;
                double real_freq = imu_count / dt_sec;
                
                RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "imu freq: %f", real_freq);
                last_imu_ts = ts;
                imu_count = 0;
            }
        }

        // ========== compute imu delta time ==========


        sensor_base::RawImuPoint *imu = (sensor_base::RawImuPoint *)data->data;
        {
            std::lock_guard<std::timed_mutex> lock(data_mutex_);
            data_1_.gyro_data.gyro[0] = imu->gyro_x * config_.RotationMatrix[0][0] +
                                        imu->gyro_y * config_.RotationMatrix[0][1] +
                                        imu->gyro_z * config_.RotationMatrix[0][2];
            
            data_1_.gyro_data.gyro[1] = imu->gyro_x * config_.RotationMatrix[1][0] +
                                        imu->gyro_y * config_.RotationMatrix[1][1] +
                                        imu->gyro_z * config_.RotationMatrix[1][2];
            
            data_1_.gyro_data.gyro[2] = imu->gyro_x * config_.RotationMatrix[2][0] +
                                        imu->gyro_y * config_.RotationMatrix[2][1] +
                                        imu->gyro_z * config_.RotationMatrix[2][2];
            
            data_1_.accel_data.accel[0] =   imu->acc_x * config_.RotationMatrix[0][0] +
                                            imu->acc_y * config_.RotationMatrix[0][1] +
                                            imu->acc_z * config_.RotationMatrix[0][2];
            
            data_1_.accel_data.accel[1] =   imu->acc_x * config_.RotationMatrix[1][0] +
                                            imu->acc_y * config_.RotationMatrix[1][1] +
                                            imu->acc_z * config_.RotationMatrix[1][2];
            
            data_1_.accel_data.accel[2] =   imu->acc_x * config_.RotationMatrix[2][0] +
                                            imu->acc_y * config_.RotationMatrix[2][1] +
                                            imu->acc_z * config_.RotationMatrix[2][2];
            

            data_1_.gyro_data.header.timestamp_nanos = ts;
            data_1_.gyro_data.header.update_count++;
            data_1_.accel_data.header.timestamp_nanos = ts;
            data_1_.accel_data.header.update_count++;
        }
        return;

    } else {
        // std::cout << "point cloud data" << std::endl;


        // ====================================================================
        // [车规级：点云完整帧 (Frame) 切帧、测时与测频逻辑]
        // 强烈建议将以下 static 变量移至 .hpp 的 private 成员中，以支持多雷达实例
        // ====================================================================
        static uint64_t current_frame_start_ts = 0;
        static uint64_t last_frame_report_ts = 0;
        static uint32_t aggregated_frame_count = 0;
        
        // 1. 根据配置的发布频率计算一帧的理论周期 (纳秒)
        // 例如：publish_freq = 10.0 Hz, 周期 = 100,000,000 ns (100 ms)
        uint64_t frame_period_ns = static_cast<uint64_t>(1e9 / config_.publish_freq);

        if (current_frame_start_ts == 0) {
            current_frame_start_ts = ts;
            last_frame_report_ts = ts;
        }

        // [车规级防御 1]：时间戳跳变保护 (Time Jump Protection)
        // 如果系统进行了 PTP/NTP 时钟同步，导致时间戳突然回拨，必须重置统计，否则会导致长达几十年的死锁
        if (ts < current_frame_start_ts || ts < last_frame_report_ts) {
            RCLCPP_WARN(rclcpp::get_logger("Mid360LidarSensor"), "Time jump detected! Resetting frame stats.");
            current_frame_start_ts = ts;
            last_frame_report_ts = ts;
            aggregated_frame_count = 0;
        }

        // 2. 检查当前包的时间戳是否跨越了“一帧”的边界
        if (ts - current_frame_start_ts >= frame_period_ns) {
            // ================= 【一帧攒满了】 =================
            aggregated_frame_count++;
            
            // [车规级测时]：计算这“一帧”真实的跨度耗时 (毫秒)
            double actual_frame_time_ms = static_cast<double>(ts - current_frame_start_ts) / 1e6;
            
            // [车规级防御 2]：严格周期推进 (消除累积误差)
            // 不要用 current_frame_start_ts = ts; 因为 ts 可能略微超出一帧的理论时间点，
            // 每次直接赋值 ts 会导致累积误差，最终导致掉帧。加上理论周期才是最准的。
            current_frame_start_ts += frame_period_ns; 
            
            // 3. [车规级测频]：每 1 秒在终端打印一次真实的帧率
            if (ts - last_frame_report_ts >= 1000000000ULL) {
                double dt_sec = static_cast<double>(ts - last_frame_report_ts) / 1e9;
                double real_frame_freq = aggregated_frame_count / dt_sec;
                
                // 打印绿色高亮：帧频率与实际耗时
                RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), 
                            "\033[32m[Mid360] PointCloud Frame Freq: %.2f Hz | Frame Time: %.2f ms\033[0m", 
                            real_frame_freq, actual_frame_time_ms);
                
                last_frame_report_ts = ts;
                aggregated_frame_count = 0;
            }

            // TODO: 这里是触发上层“帧处理”的绝佳时机！
            // 比如通知 condition_variable，或者翻转 data_1_ 和 data_2_ 的标志位
        }
        // ====================================================================

        RawPacket pkt = {};
        pkt.handle = handle;
        pkt.lidar_type = LidarProtoType::kLivoxLidarType;
        pkt.extrinsic_enable = false;
        if(dev_type == LivoxLidarDeviceType::kLivoxLidarTypeIndustrialHAP) {
            pkt.line_num = kLineNumberHAP;
        } else if (dev_type == LivoxLidarDeviceType::kLivoxLidarTypeMid360) {
            pkt.line_num = kLineNumberMid360;
        } else {
            pkt.line_num = kLineNumberDefault;
        }
        pkt.data_type = data->data_type;
        pkt.point_num = data->dot_num;

        // [车规级防御 3]：除零保护。如果由于干扰产生了一个空包 (dot_num = 0)，程序不能崩溃
        pkt.point_interval = (data->dot_num == 0) ? 0 : (data->time_interval * 100 / data->dot_num); // ns
        // pkt.point_interval = data->time_interval * 100 / data->dot_num; // ns

        // todo: fix this
        // pkt.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp,
                                            //   sizeof(data->timestamp));

        uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
        pkt.raw_data.insert(pkt.raw_data.end(), data->data,
                           data->data + length);
        {
            std::lock_guard<std::timed_mutex> lock(data_mutex_);
            
            // todo: push into queue
        }
        return;
    }
}


} // namespace sensor_base



// typedef struct {
//   uint8_t version;
//   uint16_t length;
//   uint16_t time_interval;      /**< unit: 0.1 us */
//   uint16_t dot_num;
//   uint16_t udp_cnt;
//   uint8_t frame_cnt;
//   uint8_t data_type;
//   uint8_t time_type;
//   uint8_t rsvd[12];
//   uint32_t crc32;
//   uint8_t timestamp[8];
//   uint8_t data[1];             /**< Point cloud data. */
// } LivoxLidarEthernetPacket;


// typedef struct {
//   LidarProtoType lidar_type;
//   uint32_t handle;
//   bool extrinsic_enable;
//   uint32_t point_num;
//   uint8_t data_type;
//   uint8_t line_num;
//   uint64_t time_stamp;
//   uint64_t point_interval;
//   std::vector<uint8_t> raw_data;
// } RawPacket;