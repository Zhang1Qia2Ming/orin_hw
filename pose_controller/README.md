
# pose_controller

`pose_controller` is a ROS 2 `ros2_control` controller plugin that converts shared-memory pose streams into published `nav_msgs/msg/Odometry` messages. It subscribes to hardware state interfaces that expose `sensor_base::PoseDataLayout` pointers, then publishes per‑stream odometry topics using worker threads.

## 1. Dependencies

Required packages and libraries:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `controller_interface`, `hardware_interface`, `pluginlib`.
*   **Parameters**: `generate_parameter_library` (generates `pose_controller_parameters`).
*   **Realtime / Utilities**: `realtime_tools`.
*   **Messages**: `std_msgs`, `sensor_msgs`, `nav_msgs`, `geometry_msgs`.
*   **Transforms**: `tf2_ros`.
*   **Shared Data Layouts**: `sensor_base`.

## 2. Contents of the .h and .cpp files

*   **`include/pose_controller/pose_controller.hpp`**
	*   **Purpose**: Declares the `PoseController` plugin and its supporting data structures.
	*   **Key Elements**:
		*   `PosePublishTask` and `PoseStreamContext` hold queued pose updates and per‑topic state.
		*   `PoseController` inherits from `controller_interface::ControllerInterface` and overrides lifecycle hooks, interface configuration, and `update()`.
		*   Manages a worker thread per pose stream, a parameter listener (`ParamListener`), and an atomic `is_running_` flag.
		*   Plugin registration via `PLUGINLIB_EXPORT_CLASS` so `controller_manager` can load it.

*   **`src/pose_controller.cpp`**
	*   **Purpose**: Implements the controller behavior and publishing logic.
	*   **Key Elements**:
		*   `on_init()` loads generated parameters.
		*   `on_configure()` creates a `PoseStreamContext` for each entry in `pose_list`, and sets up the odometry publishers.
		*   `on_activate()` starts a worker thread per stream; `on_deactivate()` stops threads and clears queues.
		*   `update()` reads the pose pointer from each state interface, checks `update_count`, and enqueues a `PosePublishTask`.
		*   `worker_thread()` converts the queued data to `nav_msgs::msg::Odometry` and publishes it.

## 3. Parameter configuration analysis

Parameters are generated from `src/pose_controller_parameters.yaml` via `generate_parameter_library`, then loaded by `ParamListener` in `on_init()`/`on_configure()`.

Key configuration concepts:
*   **`pose_list`**: List of hardware state interface names to subscribe to. Each entry defines a pose stream and becomes the base topic name.

Behavior driven by parameters:
*   **Topic naming**: Each pose stream publishes odometry to the topic named exactly by its interface name.
*   **Frame IDs**: The controller sets `frame_id` to `odom`, and `child_frame_id` is derived from the interface name prefix (before the `/`).
*   **Update gating**: Publishing only occurs when the underlying `PoseDataLayout::header.update_count` increases, ensuring new data per message.

If you add a new pose stream, make sure its interface name is listed in `pose_list` and matches the hardware interface naming scheme.
