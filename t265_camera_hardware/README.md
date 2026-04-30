# t265_camera_hardware

`t265_camera_hardware` is a ROS 2 hardware interface package that bridges the `t265_camera_base` driver with the `ros2_control` framework. By wrapping the sensor logic in a `hardware_interface::SystemInterface`, it exposes the Intel RealSense T265's tracking and imaging streams as standardized state interfaces, making it accessible to controllers within `ros2_control`.

## 1. Dependencies

Building and running this package requires the following packages and system components:
*   **ROS 2 Ecosystem**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control Framework**: `hardware_interface`, `pluginlib` (required to build and export the hardware component as a dynamically loadable plugin).
*   **Device Base Libraries**: `sensor_base` and `t265_camera_base` (which contain the underlying RealSense SDK handling and lock-free thread structures).
*   **Vision Dependencies**: `OpenCV` and `cv_bridge`.

## 2. Contents of the .h and .cpp files

*   **`include/t265_camera_hardware/t265_camera_hw_interface.hpp`**
    *   **Purpose**: Declares the `T265CameraHwInterface` class, extending the standard `hardware_interface::SystemInterface` from `ros2_control`.
    *   **Key Elements**:
        *   Overrides essential `ros2_control` lifecycle functions: `on_init()`, `on_configure()`, `on_activate()`, and `on_deactivate()`.
        *   Provides `export_state_interfaces()` to expose the camera's outputs (pose, IMU, fisheye images) to downstream controllers.
        *   Declares the main control loops: `read()` (to pull sensor updates) and `write()`.
        *   Manages a pointer to the low-level `sensor_base::T265CameraSensor` class.

*   **`include/t265_camera_hardware/sensor_base_types.hpp`**
    *   **Purpose**: Helper header that likely maps or includes definitions related to integrating the strict real-time types with `hardware_interface::StateInterface`.

*   **`src/t265_camera_hw_interface.cpp`**
    *   **Purpose**: Contains the implementations matching the `ros2_control` protocol.
    *   **Key Elements**:
        *   `on_init()`: Parses node/URDF properties via `HardwareInfo` (e.g., `serial_no`, `usb_port_id`, `initial_reset`) and instantiates `T265CameraSensor` with the exact configuration.
        *   `on_activate()` & `on_deactivate()`: Responsible for safely starting and stopping the background polling threads within the hardware driver.
        *   `read()`: Continually executes within the real-time loop. It securely locks and fetches the double-buffered structs (pose, gyroscope, accelerometer, and images) managed by `t265_camera_base` and propagates those values to the exposed state interfaces for the given period.
        *   Utilizes the `PLUGINLIB_EXPORT_CLASS` macro to expose the class as a plugin, allowing `controller_manager` to pick it up dynamically at runtime.
