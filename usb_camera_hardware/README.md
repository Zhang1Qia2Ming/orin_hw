# usb_camera_hardware

`usb_camera_hardware` is a ROS 2 package that wraps the `usb_camera_base` driver into a standard `ros2_control` hardware interface. This allows a generic USB camera to be managed, started, and read from within the `ros2_control` framework (specifically via `hardware_interface::SystemInterface`).

## 1. Dependencies

To build and run this package, the following dependencies are required:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `hardware_interface`, `pluginlib` (to export the class as a dynamically loadable hardware plugin).
*   **Sensor & Hardware Libraries**: `sensor_base` and `usb_camera_base` (the low-level driver communicating with the actual camera).
*   **Third-party Libraries**: `OpenCV`, `cv_bridge`.

## 2. Contents of the .h and .cpp files

*   **`include/usb_camera_hardware/usb_camera_hw_interface.hpp`**
    *   **Purpose**: Declares the `UsbCameraHwInterface` class which implements the `hardware_interface::SystemInterface`.
    *   **Key Elements**:
        *   Inheritance from `hardware_interface::SystemInterface`.
        *   Declaration of standard `ros2_control` lifecycle methods: `on_init()`, `on_configure()`, `on_activate()`, `on_deactivate()`.
        *   Resource exporting functions: `export_state_interfaces()` (to expose image data/states to controllers) and `export_command_interfaces()`.
        *   Real-time loop functions: `read()` and `write()`.
        *   A shared pointer to the `sensor_base::UsbCameraSensor` (from `usb_camera_base`) to interact with the physical device.

*   **`src/usb_camera_hw_interface.cpp`**
    *   **Purpose**: Implements the plugin defined in the header, mapping the `ros2_control` lifecycle to the underlying `UsbCameraSensor`.
    *   **Key Elements**:
        *   **`on_init()`**: Parses the `HardwareInfo` (from the URDF/Xacro) to populate parameters like `video_device`, width, and framerate. Instantiates the `UsbCameraSensor`.
        *   **`on_activate()`**: Starts the camera thread and enables the data stream.
        *   **`on_deactivate()`**: Safely halts the camera thread and closes the device.
        *   **`read()`**: Periodically checks the double buffers (e.g., `data_2_ptr_`) populated by the underlying `UsbCameraSensor` plugin to fetch the latest camera frame and push it to the exposed `ros2_control` state interfaces.
        *   **Plugin Registration**: Uses `PLUGINLIB_EXPORT_CLASS` macro at the bottom so the controller manager can discover and load this interface dynamically at runtime.
