
# lidar_controller

`lidar_controller` is a ROS 2 `ros2_control` controller plugin that reads LiDAR data from hardware state interfaces and publishes both Livox custom messages and standard `sensor_msgs/msg/PointCloud2`. It optionally fuses IMU data to provide motion‑compensated point cloud output.

## 1. Dependencies

Required packages and libraries:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `controller_interface`, `hardware_interface`, `pluginlib`.
*   **Parameters**: `generate_parameter_library` (generates `lidar_controller_parameters`).
*   **Realtime / Utilities**: `realtime_tools`.
*   **Messages**: `std_msgs`, `sensor_msgs`, `controller_msg`, `livox_ros_driver2`.
*   **Vision**: `cv_bridge` (only used for conversion utilities in this controller).
*   **Math**: `Eigen3`.
*   **Shared Data Layouts**: `sensor_base`.

## 2. Contents of the .h and .cpp files

*   **`include/lidar_controller/lidar_controller.hpp`**
	*   **Purpose**: Declares the `LidarController` plugin and its stream context types.
	*   **Key Elements**:
		*   `LidarPublishTask` and `ImuPublishTask` hold queued LiDAR/IMU samples.
		*   `LidarStreamContext` maintains per‑stream publishers, queues, extrinsics, and IMU integration helpers.
		*   `LidarController` inherits from `controller_interface::ControllerInterface` and overrides lifecycle hooks, interface configuration, and `update()`.
		*   Plugin registration via `PLUGINLIB_EXPORT_CLASS` for controller discovery.

*   **`include/lidar_controller/IMU_integeator.hpp`**
	*   **Purpose**: Lightweight IMU pre‑integration logic for motion compensation.
	*   **Key Elements**: Bias estimation, gravity alignment, and incremental pose propagation (`Predict`, `GetCurrentPose`).

*   **`include/lidar_controller/lock_free_pose_buffer.hpp`**
	*   **Purpose**: Lock‑free ring buffer for IMU‑integrated pose snapshots.
	*   **Key Elements**: `Push()` for high‑rate IMU writes and `QueryInterpolatedPose()` for time‑aligned LiDAR pose retrieval.

*   **`src/lidar_controller.cpp`**
	*   **Purpose**: Implements the controller runtime logic and publishing pipeline.
	*   **Key Elements**:
		*   `on_init()` loads generated parameters; `on_configure()` builds per‑stream publishers and optional dynamic extrinsics subscribers.
		*   `on_activate()` starts publisher and IMU integration threads; `on_deactivate()` stops threads.
		*   `update()` reads LiDAR pointers (and IMU pointers when enabled), copies point data into worker pools, and queues publish tasks.
		*   `publish_worker()` and `imu_integration_worker()` handle data conversion, motion compensation, and message publication.

## 3. Parameter configuration analysis

Parameters are generated from `src/lidar_controller_parameters.yaml` via `generate_parameter_library` and loaded by `ParamListener` in `on_init()`/`on_configure()`.

Key configuration concepts:
*   **`lidar_list`**: List of hardware state interface names to subscribe to. Each entry defines a LiDAR stream and becomes the base topic name.
*   **`lidar_config.lidar_list_map`**: Per‑stream configuration map keyed by the same interface name. Common fields include:
    *   `frame_id` for published messages.
    *   `enable_dynamic_extrinsics` to subscribe to `<interface_name>/extrinsics` and update extrinsic transforms at runtime.
    *   `enable_imu_integration` to fuse IMU data for motion compensation.

Behavior driven by parameters:
*   **Topic naming**: Each stream publishes `<interface_name>_custom_msg` and `<interface_name>_point_cloud`.
*   **IMU integration**: When enabled, the controller derives an IMU interface name by replacing `/lidar` with `/imu` (or appending `/imu`) and reads from that state interface.
*   **Dynamic extrinsics**: When enabled, the controller listens on `<interface_name>/extrinsics` and updates the transform used during point cloud generation.

If you add new LiDARs, ensure each interface name is listed in `lidar_list` and has a matching entry in `lidar_list_map`.
