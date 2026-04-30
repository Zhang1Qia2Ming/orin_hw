
# image_controller

`image_controller` is a ROS 2 `ros2_control` controller plugin that reads shared‑memory image streams from hardware state interfaces and publishes them as ROS image topics. It supports raw, undistorted, and JPEG‑compressed outputs using per‑stream configuration.

## 1. Dependencies

Required packages and libraries:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `controller_interface`, `hardware_interface`, `pluginlib`.
*   **Parameters**: `generate_parameter_library` (generates `image_controller_parameters`).
*   **Realtime / Utilities**: `realtime_tools`.
*   **Messages**: `std_msgs`, `sensor_msgs`.
*   **Vision**: `cv_bridge` (OpenCV bridge).
*   **Shared Data Layouts**: `sensor_base`.

## 2. Contents of the .h and .cpp files

*   **`include/image_controller/image_controller.hpp`**
	*   **Purpose**: Declares the `ImageController` plugin and the per‑stream context structures.
	*   **Key Elements**:
		*   `ImagePublishTask` and `ImageStreamContext` manage queued frames, per‑stream publishers, and processing state.
		*   `ImageController` inherits from `controller_interface::ControllerInterface` and overrides lifecycle hooks, interface configuration, and `update()`.
		*   Thread management for processing and publishing, plus `init_maps()` for undistortion map initialization.
		*   Plugin registration via `PLUGINLIB_EXPORT_CLASS`.

*   **`src/image_controller.cpp`**
	*   **Purpose**: Implements the controller runtime logic and publishing pipeline.
	*   **Key Elements**:
		*   `on_init()` loads generated parameters, `on_configure()` sets up per‑stream publishers and configuration.
		*   `on_activate()` launches processing and publishing threads; `on_deactivate()` stops threads and clears queues.
		*   `update()` reads image pointers from state interfaces (from `sensor_base::ImageDataLayout`) and enqueues frames for processing.
		*   `processor_thread()` handles undistortion and JPEG encoding; `publisher_thread()` publishes raw/undistorted/compressed topics.

## 3. Parameter configuration analysis

Parameters are generated from `src/image_controller_parameters.yaml` via `generate_parameter_library`, then loaded by `ParamListener` in `on_init()`/`on_configure()`.

Key configuration concepts:
*   **`image_list`**: List of hardware state interface names to subscribe to. Each entry defines a stream and becomes the base topic name.
*   **`image_config.image_list_map`**: Per‑stream configuration map keyed by the same interface name. For each stream you can enable:
    *   `publish_raw`
    *   `publish_undistorted`
    *   `publish_compressed`
    *   Intrinsics/Distortion data used for undistortion (`intrinsic_matrix`, `distortion`, optional `extrinsics`).

Behavior driven by parameters:
*   **Topic naming**: Each stream publishes to `<interface_name>/raw`, `<interface_name>/undistorted`, and/or `<interface_name>/compressed` depending on the flags above.
*   **Undistortion**: When `publish_undistorted` is enabled, `init_maps()` uses the configured intrinsics/distortion to build the remap tables.
*   **Compression**: When `publish_compressed` is enabled, JPEG encoding is applied before publishing.

If you add a new camera stream, ensure its interface name is listed in `image_list` and that a matching entry exists in `image_list_map`.
