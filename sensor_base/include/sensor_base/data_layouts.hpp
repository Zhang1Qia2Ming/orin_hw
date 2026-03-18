#ifndef SENSOR_BASE_DATA_LAYOUTS_HPP_
#define SENSOR_BASE_DATA_LAYOUTS_HPP_

#include <cstdint>
#include <vector>

#include <opencv2/opencv.hpp>

namespace sensor_base {

    struct alignas(16) SensorHeader {
        volatile uint64_t update_count;
        uint64_t timestamp_nanos;
    };

    struct alignas(64) PoseDataLayout {
        SensorHeader header;

        double pose[7]; // x, y, z, qw, qx, qy, qz
        double velocity[6]; // vx, vy, vz, wx, wy, wz
        double reserved[1];
    };

    struct alignas(64) GyroDataLayout {
        SensorHeader header;
        double gyro[3];
        double reserved[3];
    };

    struct alignas(64) AccelDataLayout {
        SensorHeader header;
        double accel[3];
        double reserved[3];
    };

    struct ImageDataLayout {
        SensorHeader header;
        char frame_id[64];
        uint32_t height;
        uint32_t width;
        cv::Mat image;

    };

    //lidar data layout
    struct LidarDataPointLayout {
        float x;
        float y;
        float z;
        float intensity;
        uint8_t tag;
        uint8_t line;
        uint64_t offset_time;
    };

    struct LidarDataLayout {
        SensorHeader header;
        uint64_t timestamp_nanos;
        uint32_t point_num;
        uint8_t lidar_id;
        std::vector<LidarDataPointLayout> points;
    };

}  // namespace sensor_base

#endif  // SENSOR_BASE_DATA_LAYOUTS_HPP_
