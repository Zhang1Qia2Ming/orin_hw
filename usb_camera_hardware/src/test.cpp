#include "usb_camera_hardware/usb_camera_sensor.hpp"
#include <iostream>
#include <chrono>
#include <thread>

int main(int argc, char** argv) {
    // 1. 模拟配置
    sensor_base::UsbCameraConfig config;
    config.video_device = "/dev/video0";
    config.image_width = 640;
    config.image_height = 480;
    config.framerate = 60.0;
    config.pixel_format = "mjpeg";

    // 2. 实例化传感器
    auto sensor = std::make_shared<sensor_base::UsbCameraSensor>("test_cam");

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

    // 3. 启动异步采集线程
    std::cout << "--- Starting Thread ---" << std::endl;
    sensor->start_thread(); // 这里假设你在 SensorBase 里实现了启动 thread 的逻辑

    // 4. 循环观察数据更新标志位（模拟 Controller 行为）
    uint64_t last_count = 0;
    auto start_time = std::chrono::steady_clock::now();

    int t = 5;
    std::cout << "--- Watching Data (" << t <<" seconds) ---" << std::endl;
    while (std::chrono::steady_clock::now() - start_time < std::chrono::seconds(t)) {
        // 通过我们设计的“后门”直接读标志位
        // 注意：在测试里我们直接模拟读 POD 里的 update_count
        auto current_count = sensor->data_->header.update_count;

        if (current_count != last_count) {
            std::cout << "Capture! Frame Count: " << current_count 
                      << " Timestamp: " << sensor->data_->header.timestamp_nanos 
                      << std::endl;
            last_count = current_count;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 5. 停止
    std::cout << "--- Stopping ---" << std::endl;
    sensor->stop_thread();
    sensor->close_device();

    return 0;
}