# sensor_base

`sensor_base` is a core utility ROS 2 package designed to provide foundational, reusable C++ structures and base classes for various hardware sensors in a robotics system. It sets up standard interfaces and lock-free thread mechanics for sensor drivers to inherit from, ensuring uniform integration within frameworks like `ros2_control`.

## 1. Dependencies

This package essentially works as a library and expects the following components to build:
*   **ROS 2 and CMake**: `ament_cmake`, `rclcpp`.
*   **Concurrency/Real-Time Tools**: `realtime_tools` (used for real-time safe operations).
*   **Third-party Libraries**: `OpenCV` and `cv_bridge` (for core image dependencies and layouts).

## 2. Contents of the .h and .cpp files

*   **`include/sensor_base/sensor_base.hpp` & `src/sensor_base.cpp`**
    *   **Purpose**: Defines the `SensorBase` abstract base class used to standardize sensor implementation.
    *   **Key Elements**: 
        *   Contains pure virtual functions (`init()`, `open_device()`, `close_device()`, `main_loop()`) that child classes **must** implement. 
        *   Manages the lifecycle of a dedicated background reading thread (`start_thread()`, `stop_thread()`, and standard state-tracking via `is_running_`). 
*   **`include/sensor_base/data_layouts.hpp`**
    *   **Purpose**: Centralizes the definition of real-time safe data structures used to pass hardware information between drivers and hardware interfaces.
    *   **Key Elements**: Includes structs enforced with exact memory alignment (e.g., `alignas(64)`) such as `SensorHeader`, `PoseDataLayout`, `GyroDataLayout`, and `AccelDataLayout`. These structures use atomic components (like `volatile uint64_t update_count`) to ensure lock-free visibility between concurrent hardware acquisition and control threads.
*   **`include/sensor_base/spsc_queue.hpp`**
    *   **Purpose**: Implements a high-performance, lock-free queue for multi-threading scenarios.
    *   **Key Elements**: A template `SPSCQueue` (Single-Producer, Single-Consumer) utilizing atomic `head_` and `tail_` indices along with `std::memory_order` directives. It's intended to securely pass high-frequency sensor payloads to the rest of the application without utilizing mutexes, avoiding priority inversions in real-time execution.
