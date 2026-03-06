#include "usb_camera_base/usb_camera_sensor.hpp"
#include "t265_camera_base/t265_camera_sensor.hpp"

namespace sensor_base {

class SensorFactory {
public:
    static std::unique_ptr<SensorBase> create_sensor(const std::string & sensor_type, const std::string & name) {
        if (sensor_type == "usb_camera") {
            return std::make_unique<UsbCameraSensor>(name);
        } else if (sensor_type == "t265_camera") {
            return std::make_unique<T265CameraSensor>(name);
        }

        // Add more sensor types as needed

        return nullptr;
    }
};


} // namespace sensor_base