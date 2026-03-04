#ifndef _USB_CAMERA_HARDWARE_USB_CAMERA_SENSOR_HPP_
#define _USB_CAMERA_HARDWARE_USB_CAMERA_SENSOR_HPP_

#include "sensor_base/sensor_base.hpp"
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_base/data_layouts.hpp"

namespace sensor_base {

struct UsbCameraConfig {
    std::string video_device;
    int image_width;
    int image_height;
    double framerate;
    std::string pixel_format;
    std::string camera_frame_id;
    std::string output_mode;
};

class UsbCameraSensor : public SensorBase {
public:
    std::shared_ptr<sensor_base::ImageDataLayout> data_;
    UsbCameraSensor(const std::string & name);
    ~UsbCameraSensor();

    bool init() override {return true;}
    bool init(const UsbCameraConfig & config);
    bool open_device() override;
    bool close_device() override;

protected:
    void main_loop() override;

private:
    std::string name_;
    cv::VideoCapture cap_;
    double data_ptr_address_ = 0.0;

    std::mutex data_mutex_;

    // config
    UsbCameraConfig config_;

};

} // namespace sensor_base


#endif // _USB_CAMERA_HARDWARE_USB_CAMERA_SENSOR_HPP_