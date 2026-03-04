#ifndef SENSOR_BASE_HPP_
#define SENSOR_BASE_HPP_

#include <string>
#include <thread>

#include "sensor_base/data_layouts.hpp"

namespace sensor_base {

class SensorBase {
    public:
        SensorBase(std::string sensor_name) : sensor_name_(sensor_name), is_running_(false) {}
        virtual ~SensorBase() { stop_thread(); }

        virtual bool init() = 0;
        virtual void start_thread();
        virtual void stop_thread();
        std::string get_name() const;
        virtual bool open_device() = 0;
        virtual bool close_device() = 0;

    protected:      
        virtual void main_loop() = 0;

        std::string sensor_name_;
        bool is_running_ = false;
        std::thread thread_ptr_;

        std::unique_ptr<SensorHeader> shared_data_ = std::make_unique<SensorHeader>();
};

}  // namespace sensor_base

#endif  // SENSOR_BASE_HPP_
