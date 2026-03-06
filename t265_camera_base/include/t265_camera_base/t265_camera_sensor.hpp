#ifndef _T265_CAMERA_BASE_T265_CAMERA_SENSOR_HPP_
#define _T265_CAMERA_BASE_T265_CAMERA_SENSOR_HPP_

#include "sensor_base/sensor_base.hpp"
#include "sensor_base/data_layouts.hpp"
#include "rclcpp/rclcpp.hpp"
#include <librealsense2/rs.hpp>

#include <thread>
#include <atomic>
#include <regex>
#include <memory>


namespace sensor_base {

struct T265CameraConfig {
    std::string serial_no;
    std::string usb_port_id;
    std::string device_type;
    double wait_for_device_timeout;
    double reconnect_timeout;
    bool initial_reset;
};

struct T265CameraData {
    PoseDataLayout pose;
    ImuDataLayout imu;
    ImageDataLayout fisheye0;  // left
    ImageDataLayout fisheye1;  // right
};

class T265CameraSensor : public SensorBase {
public:
    std::shared_ptr<T265CameraData> data_;
    T265CameraSensor(const std::string & name);
    ~T265CameraSensor();

    bool init() override {return true;}
    bool init(const T265CameraConfig & config);
    bool open_device();
    bool close_device();

protected:
    void main_loop() override;
            
private:
    std::string parseUsbPort(std::string line)
        {
            std::string port_id;
            std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
            std::smatch base_match;
            bool found = std::regex_match(line, base_match, self_regex);
            if (found)
            {
                port_id = base_match[1].str();
                if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
                {
                    std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
                    bool found_end = std::regex_match(port_id, base_match, end_regex);
                    if (found_end)
                    {
                        port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
                    }
                }
            }
            return port_id;
        }

        void frame_callback(const rs2::frame& f);

        void data_callback(const rs2::frame& f);

private:
    std::string name_;
    bool is_alive_ = false;
    bool is_streaming_ = false;
    rs2::device dev_;
    rs2::context ctx_;
    rs2::sensor rs_sensor_;

    std::thread query_thread_;
    
    // config
    T265CameraConfig config_;
};

} // namespace sensor_base

#endif // _T265_CAMERA_BASE_T265_CAMERA_SENSOR_HPP_