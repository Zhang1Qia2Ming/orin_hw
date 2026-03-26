#include "mid360_lidar_base/mid360_lidar_sensor.hpp"
#include "sensor_base/sensor_base.hpp"
#include "mid360_lidar_base/comm.hpp"
// #include "mid360_lidar_base/pub_handler.h"
#include <algorithm>

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
    write_buffer_ptr_ = &data_1_.lidar_data;
    ready_buffer_ptr_ = &data_2_.lidar_data;
    read_buffer_ptr_ = &data_3_.lidar_data;

    imu_ptr_pack_.gyro_ptr = &(data_2_.gyro_data);
    imu_ptr_pack_.accel_ptr = &(data_2_.accel_data);

    ImuPointerPack* pack_ptr = &imu_ptr_pack_;
    std::memcpy(&imu_ptr_as_double_, &pack_ptr, sizeof(pack_ptr));
    // pointer

    config_ = config;
    data_1_ = Mid360LidarData();
    data_1_.lidar_data.header.update_count = 0;
    data_1_.gyro_data.header.update_count = 0;
    data_1_.accel_data.header.update_count = 0;
    data_2_ = Mid360LidarData();
    data_2_.lidar_data.header.update_count = 0;
    data_2_.gyro_data.header.update_count = 0;
    data_2_.accel_data.header.update_count = 0;
    data_3_ = Mid360LidarData();
    data_3_.lidar_data.header.update_count = 0;
    data_3_.gyro_data.header.update_count = 0;
    data_3_.accel_data.header.update_count = 0;

    set_extrinsic_parameter();

    lidar_data_queue_.storage_packet = nullptr;
    lidar_data_queue_.rd_idx = 0;
    lidar_data_queue_.wr_idx = 0;

    return true;
}

bool Mid360LidarSensor::open_device()
{

    is_running_.store(true);
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
    process_thread_ = std::thread(&Mid360LidarSensor::RawDataProcess, this);


    return true;
}

bool Mid360LidarSensor::close_device()
{
    // LivoxLidarRemovePointCloudObserver();
    is_running_.store(false);
    packet_condition_.notify_all();
    if(process_thread_.joinable()) {
        process_thread_.join();
    }
    return true;
}

void Mid360LidarSensor::main_loop()
{
    // todo
}

bool Mid360LidarSensor::update_buffer2()
{
    // copy imu data
    if(data_mutex_.try_lock_for(std::chrono::microseconds(50))) {
        data_2_.gyro_data = data_1_.gyro_data;
        data_2_.accel_data = data_1_.accel_data;
        data_mutex_.unlock();
        return true;
    }
    return false;
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

    // RCLCPP_INFO(rclcpp::get_logger(name_), "RotationMatrix:[%f, %f, %f, %f, %f, %f, %f, %f, %f,]", config_.RotationMatrix[0][0], config_.RotationMatrix[0][1], config_.RotationMatrix[0][2], config_.RotationMatrix[1][0], config_.RotationMatrix[1][1], config_.RotationMatrix[1][2], config_.RotationMatrix[2][0], config_.RotationMatrix[2][1], config_.RotationMatrix[2][2]);
    
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
    // std::memcpy(&ts, data->timestamp, sizeof(data->timestamp));
    auto now = std::chrono::system_clock::now();
    ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();

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
                
                // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "imu freq: %f", real_freq);
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

        static std::vector<RawPacket> local_frame_buffer_;
        
        // 1. 根据配置的发布频率计算一帧的理论周期 (纳秒)
        // 例如：publish_freq = 10.0 Hz, 周期 = 100,000,000 ns (100 ms)
        uint64_t frame_period_ns = static_cast<uint64_t>(1e9 / config_.publish_freq);

        if (current_frame_start_ts == 0) {
            current_frame_start_ts = ts;
            last_frame_report_ts = ts;
            local_frame_buffer_.reserve(250);
        }

        // [车规级防御 1]：时间戳跳变保护 (Time Jump Protection)
        // 如果系统进行了 PTP/NTP 时钟同步，导致时间戳突然回拨，必须重置统计，否则会导致长达几十年的死锁
        if (ts < current_frame_start_ts || ts < last_frame_report_ts) {
            RCLCPP_WARN(rclcpp::get_logger("Mid360LidarSensor"), "Time jump detected! Resetting frame stats.");
            current_frame_start_ts = ts;
            last_frame_report_ts = ts;
            aggregated_frame_count = 0;
            local_frame_buffer_.clear();
        }


        // 2. 
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

        // todo: fix this
        // pkt.time_stamp = GetEthPacketTimestamp(data->time_type, data->timestamp,
                                            //   sizeof(data->timestamp));
        pkt.time_stamp = ts;

        uint32_t length = data->length - sizeof(LivoxLidarEthernetPacket) + 1;
        // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "raw_data size: %d", length);    1344
        pkt.raw_data.insert(pkt.raw_data.end(), data->data, data->data + length);
        local_frame_buffer_.push_back(pkt);


        // 3. 检查当前包的时间戳是否跨越了“一帧”的边界
        if (ts - current_frame_start_ts >= frame_period_ns) {
            // ================= 【一帧攒满了】 =================
            aggregated_frame_count++;
            
            // [车规级测时]：计算这“一帧”真实的跨度耗时 (毫秒)
            double actual_frame_time_ms = static_cast<double>(ts - current_frame_start_ts) / 1e6;
            
            // [车规级防御 2]：严格周期推进 (消除累积误差)
            // 不要用 current_frame_start_ts = ts; 因为 ts 可能略微超出一帧的理论时间点，
            // 每次直接赋值 ts 会导致累积误差，最终导致掉帧。加上理论周期才是最准的。
            
            // 3. [车规级测频]：每 1 秒在终端打印一次真实的帧率
            if (ts - last_frame_report_ts >= 1000000000ULL) {
                double dt_sec = static_cast<double>(ts - last_frame_report_ts) / 1e9;
                double real_frame_freq = aggregated_frame_count / dt_sec;
                
                // 打印绿色高亮：帧频率与实际耗时
                // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), 
                //             "\033[32m[Mid360] PointCloud Frame Freq: %.2f Hz | Frame Time: %.2f ms\033[0m", 
                //             real_frame_freq, actual_frame_time_ms);
                
                last_frame_report_ts = ts;
                aggregated_frame_count = 0;
            }

            {
                // std::lock_guard<std::timed_mutex> lock(data_mutex_, std::defer_lock);
                std::unique_lock<std::mutex> lock(packet_mutex_);
                raw_packet_queue_.push_back(std::move(local_frame_buffer_));
                packet_condition_.notify_one();
            }
            current_frame_start_ts += frame_period_ns;
            local_frame_buffer_.reserve(250);
        }
        
        return;
    }
}

void Mid360LidarSensor::RawDataProcess() {
    
    while(is_running_.load()){
        std::vector<RawPacket> current_frame;
        {
            std::unique_lock<std::mutex> lock(packet_mutex_);
            packet_condition_.wait(lock, [this]() { return !raw_packet_queue_.empty() || !is_running_.load(); });
            if(!is_running_.load() && raw_packet_queue_.empty()) {
                break;
            }
            current_frame = std::move(raw_packet_queue_.front());
            raw_packet_queue_.pop_front();
        }
        write_buffer_ptr_->points.clear();
        for(auto& pkt : current_frame) {
            PointCloudProcess(pkt, write_buffer_ptr_->points);
        }
        // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "timestamp now: %ld", std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
        
        write_buffer_ptr_->point_num = write_buffer_ptr_->points.size();
        // ping pong
        {
            std::lock_guard<std::mutex> lock(buffer_swap_mutex_);
            std::swap(write_buffer_ptr_, ready_buffer_ptr_);
            is_new_frame_ready_ = true;
        }
    }
}

void Mid360LidarSensor::PointCloudProcess(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer) {
    if (raw_data.lidar_type == LidarProtoType::kLivoxLidarType) {
        LivoxLidarPointCloudProcess(raw_data, target_buffer);
    } else {
    static bool flag = false;
    if (!flag) {
      std::cout << "error, unsupported protocol type: "
                << static_cast<int>(raw_data.lidar_type) << std::endl;
      flag = true;
    }
  }
}

void Mid360LidarSensor::LivoxLidarPointCloudProcess(RawPacket& raw_data, std::vector<LidarDataPointLayout>& target_buffer) {
    switch (raw_data.data_type) {
        case kLivoxLidarCartesianCoordinateHighData:
            ProcessCartesianHighPoint(raw_data, target_buffer);   
            break;
        case kLivoxLidarCartesianCoordinateLowData:
            ProcessCartesianLowPoint(raw_data, target_buffer);
            break;
        case kLivoxLidarSphericalCoordinateData:
            ProcessSphericalPoint(raw_data, target_buffer);
            break;
        default:
            std::cout << "unknown data type: " << static_cast<int>(raw_data.data_type)
              << " !!" << std::endl;
            break;
    }
}

void Mid360LidarSensor::ProcessCartesianHighPoint(RawPacket& pkt, std::vector<LidarDataPointLayout>& target_buffer) {
    
    LivoxLidarCartesianHighRawPoint* points = reinterpret_cast<LivoxLidarCartesianHighRawPoint*>(pkt.raw_data.data());
    
    target_buffer.reserve(pkt.point_num);

    // RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "cartesian high point function");
    LidarDataPointLayout point = {};
    {
        // std::lock_guard<std::mutex> lock(points_clouds_mutex_);
        for(uint32_t i = 0; i < pkt.point_num; i++) {
            if (pkt.extrinsic_enable) {
                point.x = points[i].x / 1000.0;
                point.y = points[i].y / 1000.0;
                point.z = points[i].z / 1000.0;
            } else {    // todo:move to controller
                point.x = (points[i].x * config_.RotationMatrix[0][0] +
                            points[i].y * config_.RotationMatrix[0][1] +
                            points[i].z * config_.RotationMatrix[0][2] + config_.TranslationVector[0]) /
                            1000.0;
                point.y = (points[i].x * config_.RotationMatrix[1][0] +
                            points[i].y * config_.RotationMatrix[1][1] +
                            points[i].z * config_.RotationMatrix[1][2] + config_.TranslationVector[1]) /
                            1000.0;
                point.z = (points[i].x * config_.RotationMatrix[2][0] +
                            points[i].y * config_.RotationMatrix[2][1] +
                            points[i].z * config_.RotationMatrix[2][2] + config_.TranslationVector[2]) /
                            1000.0;
            }
            point.intensity = points[i].reflectivity;
            point.line = i % pkt.line_num;
            point.tag = points[i].tag;
            point.offset_time = pkt.time_stamp + i * pkt.point_interval;
            target_buffer.push_back(point);
        }
    }
    return;
}

void Mid360LidarSensor::ProcessCartesianLowPoint(RawPacket& pkt, std::vector<LidarDataPointLayout>& target_buffer) {
    LivoxLidarCartesianLowRawPoint* points = reinterpret_cast<LivoxLidarCartesianLowRawPoint*>(pkt.raw_data.data());
    RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "cartesian low point function");
    // todo: add cartesian low point process
}

void Mid360LidarSensor::ProcessSphericalPoint(RawPacket& pkt, std::vector<LidarDataPointLayout>& target_buffer) {
    LivoxLidarSpherPoint* points = reinterpret_cast<LivoxLidarSpherPoint*>(pkt.raw_data.data());
    RCLCPP_INFO(rclcpp::get_logger("Mid360LidarSensor"), "spherical point function");
    // todo: add spherical point process
}

void Mid360LidarSensor::SetWriteData1(bool is_write) {
    is_write_data_1_.store(is_write);
}

uint32_t Mid360LidarSensor::GetLidarPointCloudsSize(LidarDataLayout& lidar_data_block) {
    return lidar_data_block.point_num;
}

bool Mid360LidarSensor::PullFrontBufferPointer(LidarDataLayout** out_front_ptr) {
    std::unique_lock<std::mutex> lock(buffer_swap_mutex_, std::try_to_lock);
    if(!lock || !is_new_frame_ready_ || ready_buffer_ptr_ == nullptr) {
        return false;
    }
    std::swap(read_buffer_ptr_, ready_buffer_ptr_);
    is_new_frame_ready_ = false;
    *out_front_ptr = read_buffer_ptr_;

    return true;
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



// typedef struct {
//   int32_t x;            /**< X axis, Unit:mm */
//   int32_t y;            /**< Y axis, Unit:mm */
//   int32_t z;            /**< Z axis, Unit:mm */
//   uint8_t reflectivity; /**< Reflectivity */
//   uint8_t tag;          /**< Tag */
// } LivoxLidarCartesianHighRawPoint;

// typedef struct {
//   int16_t x;            /**< X axis, Unit:cm */
//   int16_t y;            /**< Y axis, Unit:cm */
//   int16_t z;            /**< Z axis, Unit:cm */
//   uint8_t reflectivity; /**< Reflectivity */
//   uint8_t tag;          /**< Tag */
// } LivoxLidarCartesianLowRawPoint;

// typedef struct {
//   uint32_t depth;
//   uint16_t theta;
//   uint16_t phi;
//   uint8_t reflectivity;
//   uint8_t tag;
// } LivoxLidarSpherPoint;

// typedef struct {
//   float x;            /**< X axis, Unit:m */
//   float y;            /**< Y axis, Unit:m */
//   float z;            /**< Z axis, Unit:m */
//   float reflectivity; /**< Reflectivity   */
//   uint8_t tag;        /**< Livox point tag   */
//   uint8_t line;       /**< Laser line id     */
//   double timestamp;   /**< Timestamp of point*/
// } LivoxPointXyzrtlt;

// typedef struct {
//   float x;
//   float y;
//   float z;
//   float intensity;
//   uint8_t tag;
//   uint8_t line;
//   uint64_t offset_time;
// } PointXyzlt;

// typedef struct {
//   uint32_t handle;
//   uint8_t lidar_type; ////refer to LivoxLidarType
//   uint32_t points_num;
//   PointXyzlt* points;
// } PointPacket;

// typedef struct {
//   uint64_t base_time[kMaxSourceLidar] {};
//   uint8_t lidar_num {};
//   PointPacket lidar_point[kMaxSourceLidar] {};
// } PointFrame;