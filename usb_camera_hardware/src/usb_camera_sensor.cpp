#include "usb_camera_hardware/usb_camera_sensor.hpp"

namespace sensor_base {

UsbCameraSensor::UsbCameraSensor(const std::string & name) 
    : SensorBase(name)
{
    name_ = name;
}

UsbCameraSensor::~UsbCameraSensor() {close_device();}

bool UsbCameraSensor::init(const UsbCameraConfig & config)
{
    config_ = config;
    data_ = std::make_shared<sensor_base::ImageDataLayout>();
    data_->header.update_count = 0;
    return true;
}

bool UsbCameraSensor::open_device()
{
    cap_.open(config_.video_device, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(rclcpp::get_logger("UsbCameraSensor"), "Failed to open camera: %s", config_.video_device.c_str());
        return false;
    }
    
    // set usb camera properties
    if (config_.pixel_format == "mjpeg") {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    } else if (config_.pixel_format == "yuyv") {
        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
    }
    
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, config_.image_width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, config_.image_height);
    cap_.set(cv::CAP_PROP_FPS, config_.framerate);

    return true;
}

bool UsbCameraSensor::close_device()
{
    if (cap_.isOpened()) {
        cap_.release();
    }
    return true;
}

void UsbCameraSensor::main_loop()
{
    cv::Mat frame;
    cv::Mat processed_frame;

    while (is_running_)
    {
        if (!cap_.read(frame)) {
            continue;
        }
        // process frame
        cv::rotate(frame, processed_frame, cv::ROTATE_180);
        data_->image = processed_frame.clone();
        data_->header.timestamp_nanos = rclcpp::Clock().now().nanoseconds();
        data_->header.update_count++;
    }
}



} // namespace sensor_base