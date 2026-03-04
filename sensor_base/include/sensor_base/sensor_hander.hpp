// #ifndef SENSOR_BASE_SENSOR_HANDLER_HPP_
// #define SENSOR_BASE_SENSOR_HANDLER_HPP_

// #include "sensor_base/data_layouts.hpp"
// #include "hardware_interface/hardware_interface.hpp"

// #include <memory>
// #include <vector>
// #include <string>

// namespace sensor_base {

// template <typename T>
// class SensorHandler {
// public:
//     explicit SensorHandler(const std::string& name) : name_(name) {
//         // 1. 核心：申请对齐的 POD 内存
//         data_ptr_ = std::make_unique<T>();
//         // 初始化头部
//         data_ptr_->base.update_count = 0;
//     }

//     virtual ~SensorHandler() = default;

//     // 获取原始指针供 Sensor 填数
//     T* get_data() { return data_ptr_.get(); }

//     // 填完数打信号
//     void notify_updated() {
//         data_ptr_->base.update_count++;
//         // 可以在这里更新 header 中的 timestamp_nanos
//     }

//     // 供 HardwareInterface 导出接口
//     std::vector<hardware_interface::StateInterface> export_interfaces() {
//         std::vector<hardware_interface::StateInterface> interfaces;

//         // 核心黑科技：把内存地址强转成 double 传出去
//         addr_as_double_ = static_cast<double>(reinterpret_cast<uintptr_t>(data_ptr_.get()));
        
//         interfaces.emplace_back(name_, "data_ptr", &addr_as_double_);
//         interfaces.emplace_back(name_, "update_count", (double*)&data_ptr_->base.update_count);

//         // 剩下的具体字段由子类去映射
//         map_specific(interfaces);
//         return interfaces;
//     }

// protected:
//     virtual void map_specific(std::vector<hardware_interface::StateInterface>& interfaces) = 0;

//     std::string name_;
//     std::unique_ptr<T> data_ptr_;
//     double addr_as_double_ = 0.0;


// };

// }  // namespace sensor_base

// #endif  // SENSOR_BASE_SENSOR_HANDLER_HPP_
