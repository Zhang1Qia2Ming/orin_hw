
# imu_controller

`imu_controller` is a ROS 2 `ros2_control` controller plugin that reads IMU data from hardware state interfaces and publishes standard `sensor_msgs/msg/Imu` topics. It supports both a single combined `/device/imu` interface or separate `/device/gyro` and `/device/accel` interfaces.

## 1. Dependencies

Required packages and libraries:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `controller_interface`, `hardware_interface`, `pluginlib`.
*   **Parameters**: `generate_parameter_library` (generates `imu_controller_parameters`).
*   **Realtime / Utilities**: `realtime_tools`.
*   **Messages**: `std_msgs`, `sensor_msgs`.
*   **Shared Data Layouts**: `sensor_base`.

## 2. Contents of the .h and .cpp files

*   **`include/imu_controller/imu_controller.hpp`**
	*   **Purpose**: Declares the `ImuController` plugin and stream context structures.
	*   **Key Elements**:
		*   `ImuPublishTask` and `ImuStreamContext` manage queued IMU samples and per‑device metadata.
		*   `ImuController` inherits from `controller_interface::ControllerInterface` and overrides lifecycle hooks, interface configuration, and `update()`.
		*   Maintains worker threads for publishing and a generated parameter listener (`ParamListener`).
		*   Plugin registration via `PLUGINLIB_EXPORT_CLASS` for `controller_manager` discovery.

*   **`src/imu_controller.cpp`**
	*   **Purpose**: Implements the controller logic and publishing loop.
	*   **Key Elements**:
		*   `on_init()` loads parameters; `on_configure()` groups state interfaces by device name and creates publishers.
		*   `on_activate()` starts per‑device worker threads; `on_deactivate()` stops threads and clears queues.
		*   `update()` reads IMU data from shared pointers (`sensor_base::ImuPointerPack` or separate gyro/accel pointers), checks `update_count`, and enqueues tasks.
		*   `worker_thread()` builds and publishes `sensor_msgs::msg::Imu` messages with timestamps from the hardware stream.

## 3. Parameter configuration analysis

The controller’s parameters are generated from `src/imu_controller_parameters.yaml` via `generate_parameter_library`. At runtime, `ParamListener` loads these parameters in `on_init()` and `on_configure()`.

Key configuration concepts:
*   **`imu_list`**: List of hardware state interface names to subscribe to. Each entry is expected to be either:
    *   A combined IMU interface like `device_name/imu`, or
    *   Separate interfaces `device_name/gyro` and `device_name/accel`.
    The controller groups entries by `device_name` and builds one publisher per device.

Behavior driven by parameters:
*   **Topic naming**: Each device publishes to `device_name/imu` based on the parsed device name.
*   **Frame ID**: For combined IMU input, the default frame is set to `livox_frame`; for split gyro/accel inputs it uses `device_name + "_frame"`.
*   **Interface selection**: If both `gyro` and `accel` are present, the controller publishes only when the gyro’s `update_count` advances, and then pairs it with the latest accel sample.

If you add new IMU devices, ensure the corresponding state interface names are listed in `imu_list` and match the expected naming pattern.
