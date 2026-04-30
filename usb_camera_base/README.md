# usb_camera_base

`usb_camera_base` is a ROS 2 package designed to handle USB camera hardware integration, likely to be used as part of a `ros2_control` hardware interface or standalone sensor runner.

## 1. Dependencies

To build and run this package, the following dependencies are required:
*   **ROS 2 and Essential Packages**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `hardware_interface`, `pluginlib` (suggesting it might be exported as a plugin or utilized within the Hardware Abstraction Layer).
*   **Sensor Utilities**: `sensor_base`, `cv_bridge`.
*   **Third-party Libraries**: `OpenCV` (for opening and processing the video stream/images).

## 2. Contents of the .h and .cpp files

*   **`include/usb_camera_base/usb_camera_sensor.hpp`**
    *   **Purpose**: The main header file defining the camera interface.
    *   **Key Elements**:
        *   Struct `UsbCameraConfig`: Stores camera initialization parameters like `video_device` (e.g., `/dev/video0`), `image_width`, `image_height`, `framerate`, `pixel_format`, and `output_mode`.
        *   Class `UsbCameraSensor`: A class that inherits from `SensorBase`. It defines the API used to start, read, and stop the camera. It uses a double-buffer strategy (`data_1_`, `data_2_`, `data_2_ptr_`) of type `sensor_base::ImageDataLayout` to safely read frames from the camera using OpenCV and pass them to downstream controllers/broadcasters without blocking.
*   **`src/usb_camera_sensor.cpp`**
    *   **Purpose**: The implementation of the `UsbCameraSensor` class.
    *   **Key Elements**: Initializes the `cv::VideoCapture` object using the provided `UsbCameraConfig`, reads frames from the USB camera at the specified framerate and format, handles potential I/O configuration logic, and safely updates the double buffer with the newest image data.

## 3. Running `test.cpp`

The `src/test.cpp` file contains a standalone main function for testing the camera driver without launching the full ROS 2 or `ros2_control` stack. It instantiates `UsbCameraSensor`, configures it for `/dev/video0` (640x480 at 60fps), and initializes the camera stream.

### Compilation and Execution Instructions

1.  **Build the Node**:
    Navigate to the root of your workspace (e.g., `~/control_ws`) and compile the package using `colcon`:
    ```bash
    cd /home/test/control_ws
    colcon build --packages-select usb_camera_base
    ```

2.  **Environment Setup**:
    Source the built workspace to add the executable to your path:
    ```bash
    source install/setup.bash
    ```
    *Ensure your USB camera is plugged in and accessible at `/dev/video0` (you may need `sudo chmod 777 /dev/video0` or to be part of the `video` group).*

3.  **Run the Test Executable**:
    The test file is built as an executable named `hd_test`. You can execute it directly or run it using `ros2 run`:
    ```bash
    ros2 run usb_camera_base hd_test
    ```
    *(Alternatively, you can run the binary directly from `./install/usb_camera_base/lib/usb_camera_base/hd_test`)*

The program will output "--- Initializing Sensor ---", attempt to connect to the camera, and read frames depending on the test script's internal loop.
