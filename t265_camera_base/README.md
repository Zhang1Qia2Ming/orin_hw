# t265_camera_base

`t265_camera_base` is a ROS 2 hardware integration package dedicated to the Intel RealSense T265 tracking camera. It acts as the foundational driver base, offering APIs to interface directly with the device's fisheye images and IMU/odometry data, designed to be easily extensible for `ros2_control` via hardware interfaces.

## 1. Dependencies

To build and run this package, the following software components and libraries are necessary:
*   **ROS 2 System**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control framework**: `hardware_interface`, `pluginlib`.
*   **Sensor Interface**: `sensor_base` (provides base layouts and core threading implementations like `SensorBase`).
*   **Vision/Image Tools**: `OpenCV`, `cv_bridge`.
*   **RealSense SDK 2.0 (`librealsense2`)**: Explicitly linked via `/usr/local/lib/librealsense2.so` and `/usr/local/include` to interact with Intel RealSense hardware.

## 2. Contents of the .h and .cpp files

*   **`include/t265_camera_base/t265_camera_sensor.hpp`**
    *   **Purpose**: Describes the core setup parameters and data structs bridging the RealSense SDK and `sensor_base` abstractions.
    *   **Key Elements**:
        *   Struct `T265CameraConfig`: Tracks hardware parameters such as `serial_no`, `usb_port_id`, and `initial_reset`.
        *   Struct `T265CameraData`: A composite data layout encompassing odometry (`PoseDataLayout`), IMU (`GyroDataLayout`, `AccelDataLayout`), and stereo fisheye images (`ImageDataLayout fisheye0/1`).
        *   Class `T265CameraSensor`: Inherits from `SensorBase`. Utilizes a double-buffering architecture (`data_1_`, `data_2_`) guarded by a `std::timed_mutex` to safely process real-time async callbacks coming from the RealSense backend, exposing atomic pointers (`pose_ptr_`, `fisheye0_ptr_`, etc.) for synchronous data reading.
*   **`src/t265_camera_sensor.cpp`**
    *   **Purpose**: Implements the initialization, stream starting, and RealSense hardware polling.
    *   **Key Elements**: 
        *   Manages the Intel RealSense `rs2::pipeline` starting routines and asynchronous frame callbacks.
        *   `init()` prepares the double buffers with default update counts.
        *   Contains the low-level logic to extract fisheye images, gyroscope, accelerometer, and 6-DOF tracking pose from the RealSense stream, routing them precisely into the unified lock-free memory layout designed in `T265CameraData`.

## 3. Running `test.cpp`

The `src/test.cpp` file is a standalone diagnostic utility intended to ensure the T265 camera is securely connected and broadcasting data (FPS, pose, etc.) prior to engaging the entire `ros2_control` node hierarchy.

### Compilation and Execution Instructions

1.  **Build the target**:
    From your ROS workspace (e.g., `/home/test/control_ws`), build the `t265_camera_base` package using `colcon`:
    ```bash
    cd /home/test/control_ws
    colcon build --packages-select t265_camera_base
    ```

2.  **Environment Setup**:
    Source the newly built overlay. Also, ensure the RealSense USB device is available (proper `udev` rules installed for Intel RealSense cameras).
    ```bash
    source install/setup.bash
    ```

3.  **Run the Test**:
    The CMake script compiles `test.cpp` as an executable named `hd_test`. Run it directly via `ros2 run`:
    ```bash
    ros2 run t265_camera_base hd_test
    ```
    This test runs an internal loop (printing diagnostic info like the `FpsCounter` measurements) to verify the device's data streams such as fisheye streams and pose tracking are operating smoothly.
