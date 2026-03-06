#include "t265_camera_base/t265_camera_sensor.hpp"
#include "sensor_base/data_layouts.hpp"

namespace sensor_base {

T265CameraSensor::T265CameraSensor(const std::string & name)
    : SensorBase(name) 
{
    name_ = name;
}

T265CameraSensor::~T265CameraSensor() {close_device();}

bool T265CameraSensor::init(const T265CameraConfig & config)
{
    config_ = config;
    data_ = std::make_shared<T265CameraData>();
    data_->pose.header.update_count = 0;
    data_->imu.header.update_count = 0;
    data_->fisheye0.header.update_count = 0;
    data_->fisheye1.header.update_count = 0;
    return true;
}

bool T265CameraSensor::open_device()
{
    is_alive_ = true;
    is_streaming_ = false;
    auto logger = rclcpp::get_logger("T265CameraSensor");

    try {
        query_thread_ = std::thread([this, logger]() {
            RCLCPP_INFO(logger, "T265 Direct Sensor thread started.");

            while (is_alive_ && !is_streaming_) {
                try {
                    // 1. 获取当前所有设备
                    auto list = ctx_.query_devices();
                    dev_ = rs2::device(); // 重置句柄

                    for (auto&& d : list) {
                        std::string sn = d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                        std::string port = parseUsbPort(d.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

                        if ((config_.serial_no.empty() || sn == config_.serial_no) && (config_.usb_port_id.empty() || port == config_.usb_port_id)) {
                            dev_ = d;
                            break;
                        }
                    }

                    if (!dev_) {
                        std::this_thread::sleep_for(std::chrono::seconds(1));
                        continue;
                    }

                    // 2. 硬件重置逻辑 (仅执行一次)
                    if (config_.initial_reset) {
                        RCLCPP_INFO(logger, "Device matched. Performing Hardware Reset...");
                        config_.initial_reset = false;
                        dev_.hardware_reset();
                        dev_ = rs2::device();
                        std::this_thread::sleep_for(std::chrono::seconds(5));
                        continue;
                    }

                    // 3. find all Sensors and match with each member variable
                    auto sensors = dev_.query_sensors();
                    int sensor_count = 0;
                    for (auto&& sensor : sensors) {
                        sensor_count++;
                        std::string sensor_name = sensor.get_info(RS2_CAMERA_INFO_NAME);\
                        RCLCPP_INFO(logger, "Sensor [%d] Found: %s", sensor_count, sensor_name.c_str());

                        if (sensor.is<rs2::depth_sensor>() || 
                        sensor.is<rs2::color_sensor>() ||
                        sensor.is<rs2::fisheye_sensor>()) {
                            RCLCPP_INFO(logger, "Depth, Color, or Fisheye Sensor Found: %s", sensor.get_info(RS2_CAMERA_INFO_NAME));
                        }
                        else if (sensor.is<rs2::pose_sensor>()) {
                            RCLCPP_INFO(logger, "Pose Sensor Found: %s", sensor.get_info(RS2_CAMERA_INFO_NAME));
                            rs_sensor_ = sensor;
                        }
                        else {
                            RCLCPP_WARN(logger, "Unknown Sensor Type: %s", sensor.get_info(RS2_CAMERA_INFO_NAME));
                        }
                    }
                    RCLCPP_INFO(logger, "Total Sensors Found: %d", sensor_count);

                    if (!rs_sensor_) {
                        RCLCPP_ERROR(logger, "Could not find Pose Sensor on this device!");
                        std::this_thread::sleep_for(std::chrono::seconds(2));
                        continue;
                    }

                    // 4. 获取并配置 Pose Profile
                    auto profiles = rs_sensor_.get_stream_profiles();
                    rs2::stream_profile pose_profile;

                    std::vector<rs2::stream_profile> target_profiles;
                    sensor_count=0;
                    for (auto& profile : profiles) {
                        sensor_count++;
                        if (profile.stream_type() == RS2_STREAM_POSE && profile.format() == RS2_FORMAT_6DOF) {
                            target_profiles.push_back(profile);
                        }
                        if (profile.stream_type() == RS2_STREAM_FISHEYE && profile.stream_index() == 1) {
                            target_profiles.push_back(profile);
                        }
                        if (profile.stream_type() == RS2_STREAM_FISHEYE && profile.stream_index() == 2) {
                            target_profiles.push_back(profile);
                        }
                        if (profile.stream_type() == RS2_STREAM_GYRO || profile.stream_type() == RS2_STREAM_ACCEL) {
                            target_profiles.push_back(profile);
                        }
                    }
                    RCLCPP_INFO(logger, "Total Pose Stream Profiles Found: %d", sensor_count);

                    for (auto& p : profiles) {
                        if (p.stream_type() == RS2_STREAM_POSE && p.format() == RS2_FORMAT_6DOF) {
                            pose_profile = p;
                            break;
                        }
                    }

                    if (!pose_profile) {
                        RCLCPP_ERROR(logger, "6DOF Pose stream profile not found!");
                        continue;
                    }

                    // 5. Direct Open & Start Pose Sensor
                    rs_sensor_.open(target_profiles);
                    rs_sensor_.start([this](rs2::frame f) {      
                        
                        // frame_callback(f);

                        data_callback(f);
                        
                    });

                    is_streaming_ = true;
                    RCLCPP_INFO(logger, "T265 Pose Sensor is now Active and Streaming.");

                } catch (const std::exception & e) {
                    RCLCPP_ERROR(logger, "Sensor API Error: %s", e.what());
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                }
            }
            RCLCPP_INFO(logger, "T265 Direct Sensor thread exiting.");
        });
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "Sensor API Error: %s", e.what());
        return false;
    }
    return true;
}

bool T265CameraSensor::close_device()
{
    auto logger = rclcpp::get_logger("T265CameraSensor");
    try {
        if (!is_streaming_) {
            RCLCPP_WARN(logger, "Sensor is not streaming, nothing to stop.");
            return true;
        }
        rs_sensor_.stop();
        rs_sensor_.close();
        is_streaming_ = false;
    } catch (const std::exception & e) {
        RCLCPP_ERROR(logger, "Sensor API Error: %s", e.what());
    }
    return true;
}

void T265CameraSensor::main_loop()
{
    // todo: now open_device() includes streaming, so no need to start_thread()
    // in future, we need make the streaming implementation into here
}

void T265CameraSensor::frame_callback(const rs2::frame& f)
{
    auto stream = f.get_profile().stream_type();
    switch (stream)
    {
        case RS2_STREAM_GYRO:
            RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Gyro Frame");
            break;
        case RS2_STREAM_ACCEL:
            RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Accel Frame");
            break;
        case RS2_STREAM_POSE:
            RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Pose Frame");
            break;
        case RS2_STREAM_FISHEYE:
            if(f.get_profile().stream_index() == 1)
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Fisheye Frame 1");
            else 
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Fisheye Frame 2");
            break;
        default:
            RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Default Frame");
            break;
    }
}

void T265CameraSensor::data_callback(const rs2::frame& f){
    uint64_t ts = static_cast<uint64_t>(f.get_timestamp() * 1000000);

    // 1.handle pose frame
    if(auto pf = f.as<rs2::pose_frame>()){
        auto pose_data = pf.get_pose_data();
        data_->pose.pose[0] = pose_data.translation.x;
        data_->pose.pose[1] = pose_data.translation.y;
        data_->pose.pose[2] = pose_data.translation.z;
        data_->pose.pose[3] = pose_data.rotation.x;
        data_->pose.pose[4] = pose_data.rotation.y;
        data_->pose.pose[5] = pose_data.rotation.z;
        data_->pose.pose[6] = pose_data.rotation.w;
        data_->pose.velocity[0] = pose_data.velocity.x;
        data_->pose.velocity[1] = pose_data.velocity.y;
        data_->pose.velocity[2] = pose_data.velocity.z;
        data_->pose.velocity[3] = pose_data.angular_velocity.x;
        data_->pose.velocity[4] = pose_data.angular_velocity.y;
        data_->pose.velocity[5] = pose_data.angular_velocity.z;
        data_->pose.header.timestamp_nanos = ts;
    }
    // 2.handle image frame
    else if(auto imgf = f.as<rs2::video_frame>()){
        int index = imgf.get_profile().stream_index();
        if(index == 1) {
            data_->fisheye0.image = cv::Mat(imgf.get_height(), imgf.get_width(), CV_8UC1, (void*)imgf.get_data()).clone();
            data_->fisheye0.header.timestamp_nanos = ts;
        }
        else if(index == 2) {
            data_->fisheye1.image = cv::Mat(imgf.get_height(), imgf.get_width(), CV_8UC1, (void*)imgf.get_data()).clone();
            data_->fisheye1.header.timestamp_nanos = ts;
        }
    }
    // 3.handle imu frame
    else if(auto mf = f.as<rs2::motion_frame>()){
        if(mf.get_profile().stream_type() == RS2_STREAM_GYRO && mf.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            data_->imu.gyro[0] = mf.get_motion_data().x;
            data_->imu.gyro[1] = mf.get_motion_data().y;
            data_->imu.gyro[2] = mf.get_motion_data().z;
        }
        else if(mf.get_profile().stream_type() == RS2_STREAM_ACCEL && mf.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
            data_->imu.accel[0] = mf.get_motion_data().x;
            data_->imu.accel[1] = mf.get_motion_data().y;
            data_->imu.accel[2] = mf.get_motion_data().z;
        }
        data_->imu.header.timestamp_nanos = ts;
    }
}



} // namespace sensor_base


// v 2.53.1 example

    // auto profile = pipe.start(cfg, [&](rs2::frame frame)
    // {
    //     // Cast the frame that arrived to motion frame
    //     auto motion = frame.as<rs2::motion_frame>();
    //     // If casting succeeded and the arrived frame is from gyro stream
    //     if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    //     {
    //         // Get the timestamp of the current frame
    //         double ts = motion.get_timestamp();
    //         // Get gyro measures
    //         rs2_vector gyro_data = motion.get_motion_data();
    //         // Call function that computes the angle of motion based on the retrieved measures
    //         algo.process_gyro(gyro_data, ts);
    //     }
    //     // If casting succeeded and the arrived frame is from accelerometer stream
    //     if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    //     {
    //         // Get accelerometer measures
    //         rs2_vector accel_data = motion.get_motion_data();
    //         // Call function that computes the angle of motion based on the retrieved measures
    //         algo.process_accel(accel_data);
    //     }
    // });


    // auto pose = f.get_pose_data();
    //     std::stringstream ss;
    //     ss << "Pos (meter): \t\t" << std::fixed << std::setprecision(2) << pose.translation.x << ", " << pose.translation.y << ", " << pose.translation.z;
    //     draw_text(int(0.05f * r.w), int(0.2f * r.h), ss.str().c_str());
    //     ss.clear(); ss.str("");
    //     ss << "Orient (quaternion): \t" << pose.rotation.x << ", " << pose.rotation.y << ", " << pose.rotation.z << ", " << pose.rotation.w;
    //     draw_text(int(0.05f * r.w), int(0.3f * r.h), ss.str().c_str());
    //     ss.clear(); ss.str("");
    //     ss << "Lin Velocity (m/sec): \t" << pose.velocity.x << ", " << pose.velocity.y << ", " << pose.velocity.z;
    //     draw_text(int(0.05f * r.w), int(0.4f * r.h), ss.str().c_str());
    //     ss.clear(); ss.str("");
    //     ss << "Ang. Velocity (rad/sec): \t" << pose.angular_velocity.x << ", " << pose.angular_velocity.y << ", " << pose.angular_velocity.z;
    //     draw_text(int(0.05f * r.w), int(0.5f * r.h), ss.str().c_str());