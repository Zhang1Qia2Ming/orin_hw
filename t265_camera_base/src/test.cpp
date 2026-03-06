#include "t265_camera_base/t265_camera_sensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>
#include <iomanip>
#include <string>
#include <vector>

using namespace std::chrono;

// 简单的频率统计类
class FpsCounter {
public:
    void tick() {
        count_++;
        auto now = steady_clock::now();
        auto elapsed = duration_cast<milliseconds>(now - last_time_).count();
        if (elapsed >= 1000) {
            fps_ = count_ / (elapsed / 1000.0);
            count_ = 0;
            last_time_ = now;
        }
    }
    double get_fps() const { return fps_; }
private:
    uint64_t count_ = 0;
    double fps_ = 0.0;
    steady_clock::time_point last_time_ = steady_clock::now();
};


int main(int argc, char** argv) {
    // 1. 参数解析
    std::string mode = "pose"; 
    if (argc > 1) mode = argv[1];

    std::cout << "[TEST] Starting Test Mode: " << mode << std::endl;
    
    // 2. 模拟配置
    sensor_base::T265CameraConfig config;
    config.serial_no = "224622110353";
    config.usb_port_id = "4-1";
    config.device_type = "t265_camera";
    config.wait_for_device_timeout = -1.0;
    config.reconnect_timeout = 6.0;
    config.initial_reset = true;

    // 3. 实例化传感器
    auto sensor = std::make_shared<sensor_base::T265CameraSensor>("test_cam");

    std::cout << "--- Initializing Sensor ---" << std::endl;
    if (!sensor->init(config)) {
        std::cerr << "Init failed!" << std::endl;
        return -1;
    }

    std::cout << "--- Opening Device ---" << std::endl;
    if (!sensor->open_device()) {
        std::cerr << "Open device failed!" << std::endl;
        return -1;
    }

    // 4. 启动异步采集线程
    std::cout << "--- Starting Thread ---" << std::endl;
    sensor->start_thread();

    // 5. 循环观察数据更新标志位（模拟 Controller 行为）
    uint64_t last_ts = 0;
    FpsCounter fps;
    auto start_run = steady_clock::now();

    std::cout << "--- Capturing Data (Press Ctrl+C to Stop) ---" << std::endl;

    while (duration_cast<seconds>(steady_clock::now() - start_run).count() < 30) {
        if (true) {
            auto data = sensor->data_1_; // 确保你类里有这个访问接口
            uint64_t current_ts = 0;

            // 根据参数决定观测哪个字段
            if (mode == "pose") current_ts = data.pose.header.timestamp_nanos;
            else if (mode == "gyro") current_ts = data.gyro.header.timestamp_nanos;
            else if (mode == "accel") current_ts = data.accel.header.timestamp_nanos;
            else if (mode == "fisheye0") current_ts = data.fisheye0.header.timestamp_nanos;
            else if (mode == "fisheye1") current_ts = data.fisheye1.header.timestamp_nanos;

            // 检查数据是否有更新
            if (current_ts != last_ts) {
                last_ts = current_ts;
                fps.tick();

                // 打印具体数据内容
                std::cout << "\r[" << std::setw(8) << mode << "] FPS: " 
                          << std::fixed << std::setprecision(1) << fps.get_fps() << " | ";

                if (mode == "pose") {
                    std::cout << "XYZ: " << data.pose.pose[0] << ", " << data.pose.pose[1] << ", " << data.pose.pose[2];
                } else if (mode == "gyro") {
                    std::cout << "Gyro: " << data.gyro.gyro[0] << ", " << data.gyro.gyro[1] << ", " << data.gyro.gyro[2];
                } else if (mode == "accel") {
                    std::cout << "Accel: " << data.accel.accel[0] << ", " << data.accel.accel[1] << ", " << data.accel.accel[2];
                } else if (mode == "fisheye0" || mode == "fisheye1") {
                    auto& img = (mode == "fisheye0") ? data.fisheye0.image : data.fisheye1.image;
                    std::cout << "Size: " << img.cols << "x" << img.rows;
                }
                std::cout << "          " << std::flush;
            }
        }
        // 为了抓准 200Hz 的数据，循环睡眠不能太长。设为 1ms 以确保采样率足够。
        std::this_thread::sleep_for(std::chrono::microseconds(500)); 
    }

    std::cout << "\n[INFO] --- Test Completed ---" << std::endl;

    
    // 6. 停止
    std::cout << "--- Stopping ---" << std::endl;
    sensor->stop_thread();
    sensor->close_device();

    return 0;
}