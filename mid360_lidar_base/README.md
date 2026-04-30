
# mid360_lidar_base

`mid360_lidar_base` is a ROS 2 package that provides the core C++ driver layer for the Livox Mid-360 LiDAR. It handles SDK integration, packet parsing, and buffered access to point cloud and IMU data, and is designed to be reused by higher-level `ros2_control` hardware interfaces.

## 1. Dependencies

Required packages, libraries, and system components:
*   **ROS 2 Core**: `rclcpp`, `rclcpp_lifecycle`, `ament_cmake`.
*   **ros2_control**: `hardware_interface`, `pluginlib` (used for interface integration).
*   **Sensor Utilities**: `sensor_base` (data layouts and base threading).
*   **Point Cloud and Math**: `PCL`, `Eigen`.
*   **Livox SDK**: `liblivox_lidar_sdk_shared.so` and headers such as `livox_lidar_api.h` and `livox_lidar_def.h` (expected under `/usr/local/lib` and `/usr/local/include`).

## 2. Contents of the .h and .cpp files

*   **`include/mid360_lidar_base/mid360_lidar_sensor.hpp`**
	*   **Purpose**: Declares the main driver class and data structures for Mid-360 LiDAR operation.
	*   **Key Elements**:
		*   `Mid360LidarConfig`: All runtime configuration (device IP/ports, host IP/ports, data types, frame ID, extrinsics).
		*   `Mid360LidarData`: Aggregates LiDAR point data and IMU data (`LidarDataLayout`, `GyroDataLayout`, `AccelDataLayout`).
		*   `Mid360LidarSensor`: Inherits from `SensorBase`, manages SDK callbacks, packet queues, and double/triple buffering for point clouds. It exposes methods like `init()`, `open_device()`, `close_device()`, `RawDataProcess()`, and point processing routines.

*   **`include/mid360_lidar_base/comm.hpp`**
	*   **Purpose**: Declares the SDK-facing data types, queues, and helper utilities used by the driver.
	*   **Key Elements**:
		*   Livox-specific protocol enums and extrinsic parameter structs.
		*   `LidarDataQueue` and queue utility functions.
		*   `LidarImuDataQueue` class for IMU packet buffering.

*   **`src/mid360_lidar_sensor.cpp`**
	*   **Purpose**: Implements `Mid360LidarSensor` behavior, including SDK initialization, callbacks, buffering, and packet decoding.
	*   **Key Elements**: Connects to the Livox SDK, processes raw Ethernet packets, converts raw points into `LidarDataPointLayout`, and manages the ready/read buffers used by consumers.

*   **`src/comm.cpp`**
	*   **Purpose**: Implements the queue and packet utilities declared in `comm.hpp` (queue init, push/pop, and helpers).

## 3. Running `test.cpp`

The `src/test.cpp` file is a minimal standalone test that configures the driver, opens the device, and keeps the process alive.

### Compilation and Execution Instructions

1.  **Build the package**:
	From your workspace root, build only this package:
	```bash
	cd /home/test/control_ws
	colcon build --packages-select mid360_lidar_base
	```

2.  **Environment Setup**:
	Source the workspace so the test executable is available:
	```bash
	source install/setup.bash
	```

3.  **Run the test executable**:
	The test is built as `hd_test`. Run it via ROS 2:
	```bash
	ros2 run mid360_lidar_base hd_test
	```

	Ensure the LiDAR is reachable at the IPs/ports configured in `test.cpp`, and that the Livox SDK library is installed under `/usr/local/lib`.
