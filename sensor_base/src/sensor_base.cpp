#include "sensor_base/sensor_base.hpp"

namespace sensor_base {

void SensorBase::start_thread() {
    if (is_running_) {
        return;
    }
    is_running_ = true;
    thread_ptr_ = std::thread(&SensorBase::main_loop, this);
}

void SensorBase::stop_thread() {
    if (!is_running_) {
        return;
    }
    is_running_ = false;
    if (thread_ptr_.joinable()) {
        thread_ptr_.join();
    }
}

std::string SensorBase::get_name() const { return sensor_name_; }

};
