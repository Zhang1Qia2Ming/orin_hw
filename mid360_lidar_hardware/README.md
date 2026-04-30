
# mid360_lidar_hardware

`mid360_lidar_hardware` is a ROS 2 package that exposes the Mid-360 LiDAR driver (`mid360_lidar_base`) as a `ros2_control` hardware interface. It implements `hardware_interface::SystemInterface` so the controller manager can load and interact with the LiDAR through standard state interfaces.

## 1. Dependencies

Required packages and components:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `hardware_interface`, `pluginlib` (plugin export and lifecycle integration).
*   **Driver Base**: `mid360_lidar_base` (actual Livox SDK integration and packet parsing).
*   **Sensor Utilities**: `sensor_base` (shared data layouts and threading helpers).

## 2. Contents of the .h and .cpp files

*   **`include/mid360_lidar_hardware/mid360_lidar_hw_interface.hpp`**
	*   **Purpose**: Declares the `Mid360LidarHwInterface` class that implements `hardware_interface::SystemInterface`.
	*   **Key Elements**:
		*   Lifecycle overrides: `on_init()`, `on_configure()`, `on_activate()`, `on_deactivate()`.
		*   Resource exports: `export_state_interfaces()` and `export_command_interfaces()`.
		*   Real-time IO: `read()` and `write()` to expose LiDAR data to controllers.
		*   Holds a `std::unique_ptr<sensor_base::Mid360LidarSensor>` used for device operations.

*   **`include/mid360_lidar_hardware/sensor_base_types.hpp`**
	*   **Purpose**: Provides a lightweight include layer for `sensor_base` and the Mid-360 base driver types used by the interface.

*   **`src/mid360_lidar_hw_interface.cpp`**
	*   **Purpose**: Implements the `ros2_control` interface, mapping URDF parameters to driver configuration and exposing state handles.
	*   **Key Elements**:
		*   `on_init()` parses `HardwareInfo` parameters (e.g., `xfer_format`, `publish_freq`, extrinsics) and initializes `Mid360LidarSensor`.
		*   `on_activate()` starts the device and background polling threads; `on_deactivate()` stops and closes the device.
		*   `read()` pulls the latest front buffer pointer and writes it into exported state interfaces for controllers to consume.
		*   Plugin registration via `PLUGINLIB_EXPORT_CLASS` so `controller_manager` can load this hardware interface.
