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

    // LdsLidar getInstance
    DeInitQueue(&lidar_data_queue_);
    imu_data_queue_.Clear();

    connect_state_ = kConnectStateOff;
    // Register Lds

    // Initialize Lidar according to the config file
    DisableLivoxSdkConsoleLogger();

    // SDK initialization
    if (!LivoxLidarSdkInit(path.c_str())) {
        std::cout << "Failed to init livox lidar sdk." << std::endl;
        return false;
    }

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



    /* About Extrinsic Parameter */
// typedef struct {
//   float roll;  /**< Roll angle, unit: degree. */
//   float pitch; /**< Pitch angle, unit: degree. */
//   float yaw;   /**< Yaw angle, unit: degree. */
//   int32_t x;   /**< X translation, unit: mm. */
//   int32_t y;   /**< Y translation, unit: mm. */
//   int32_t z;   /**< Z translation, unit: mm. */
// } ExtParameter;

// typedef float TranslationVector[3]; /**< x, y, z translation, unit: mm. */
// typedef float RotationMatrix[3][3];

// typedef struct {
//   TranslationVector trans;
//   RotationMatrix rotation;
// } ExtParameterDetailed;

// typedef struct {
//   LidarProtoType lidar_type;
//   uint32_t handle;
//   ExtParameter param;
// } LidarExtParameter;



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

void Mid360LidarSensor::point_cloud_poll_thread(){

}

void Mid360LidarSensor::imu_poll_thread(){

}


} // namespace sensor_base
