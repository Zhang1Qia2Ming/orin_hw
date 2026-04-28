# orin_Hw

Hardware integration and sensor streaming stack for an AGX orin-based robot platform using `ros2_control`.

This workspace segment provides:
- Hardware interface plugins for USB camera, Intel T265, and Livox MID360.
- ROS 2 controllers that convert low-level shared-memory style sensor interfaces into ROS topics.
- A bringup package with launch files, URDF/xacro, and controller configuration.

---

## 1) What this repository contains

### Core packages

| Package | Role |
|---|---|
| `robot_bringup` | Launch files, URDF/xacro, controller manager params, RViz config |
| `sensor_base` | Shared sensor data layouts and common abstractions |
| `controller_msg` | Custom message definitions for Livox point cloud stream, BUT the actually it still use msgs in livox_ros_driver2 |

### Sensor hardware layers

| Package | Role |
|---|---|
| `usb_camera_base` / `usb_camera_hardware` | USB camera sensor base + ros2_control hardware interface plugin |
| `t265_camera_base` / `t265_camera_hardware` | Intel T265 sensor base + hardware interface plugin |
| `mid360_lidar_base` / `mid360_lidar_hardware` | Livox MID360 sensor base + hardware interface plugin |

### Controller plugins

| Package | Plugin | Purpose |
|---|---|---|
| `image_controller` | `image_controller/ImageController` | Publishes raw / undistorted / compressed image streams |
| `pose_controller` | `pose_controller/PoseController` | Publishes odometry (`nav_msgs/msg/Odometry`) |
| `imu_controller` | `imu_controller/ImuController` | Publishes `sensor_msgs/msg/Imu` |
| `lidar_controller` | `lidar_controller/LidarController` | Publishes Livox custom packet + point cloud |

---

## 2) Runtime architecture

At runtime, data flow is:

1. Hardware plugins expose `state_interface` values to `controller_manager`.
2. Controller plugins read those interfaces each update cycle.
3. Controllers convert raw sensor layout blocks to ROS messages.
4. ROS topics are published for downstream perception / localization / visualization.

`robot_bringup` starts:
- `ros2_control_node`
- `robot_state_publisher`
- controller spawners
- optional RViz

---

## 3) Prerequisites

Install and source a ROS 2 environment with `ros2_control` stack available.

Common dependencies used by this repository include:
- `controller_manager`, `ros2_control`, `hardware_interface`, `controller_interface`
- `pluginlib`, `rclcpp`, `rclcpp_lifecycle`, `realtime_tools`
- `OpenCV`, `cv_bridge`, `sensor_msgs`, `nav_msgs`, `tf2_ros`
- `librealsense2` (T265)
- `livox_ros_driver2`, `pcl_conversions`, PCL (MID360)

> Note: Exact package names vary by Ubuntu/ROS distribution.

---

## 4) Build instructions

From workspace root:

```bash
cd /home/test/control_ws
colcon build --symlink-install
source install/setup.bash
```

To build only this package group:

```bash
cd /home/test/control_ws
colcon build --symlink-install --packages-select \
	sensor_base controller_msg \
	usb_camera_base usb_camera_hardware \
	t265_camera_base t265_camera_hardware \
	mid360_lidar_base mid360_lidar_hardware \
	image_controller pose_controller imu_controller lidar_controller \
	robot_bringup
source install/setup.bash
```

---

## 5) Quick start

### Option A: use provided script

From workspace root:

```bash
./run.sh
```

This runs:

```bash
taskset -c 0-7 ros2 launch robot_bringup test.launch.py
```

### Option B: run directly

```bash
cd /home/test/control_ws
source install/setup.bash
ros2 launch robot_bringup test.launch.py
```

---

## 6) Launch files

| Launch file | Purpose |
|---|---|
| `test.launch.py` | Full sensor/controller bringup with RViz |
| `test_rviz.launch.py` | Same main pipeline with RViz |
| `test_lower_half.launch.py` | Minimal lower-half stack (control + state publisher, no controller spawners) |

Common launch args:
- `prefix` (default: `""`)
- `use_mock_hardware` (default: `false`)
- `mock_sensor_commands` (default: `false`)
- `slowdown` (default: `50.0`)

Example:

```bash
ros2 launch robot_bringup test.launch.py use_mock_hardware:=true
```

---

## 7) Default controller configuration

Main config file: `robot_bringup/config/test_params.yaml`

### Controller manager
- update rate: `250 Hz`
- loaded controllers:
	- `test_image_controller`
	- `test_pose_controller`
	- `test_imu_controller`
	- `test_lidar_controller`

### Default sensor interface mapping
- Image interfaces:
	- `t265_front/image0`
	- `t265_front/image1`
	- `camera_front/image`
- Pose interfaces:
	- `t265_front/pose`
- IMU interfaces:
	- `t265_front/accel`
	- `t265_front/gyro`
	- `mid360_front/imu`
- Lidar interfaces:
	- `mid360_front/lidar`

---

## 8) Published topics (default behavior)

### ImageController
For each interface `<name>`:
- `<name>/raw` (`sensor_msgs/msg/Image`) when `publish_raw=true`
- `<name>/undistorted` (`sensor_msgs/msg/Image`) when `publish_undistorted=true`
- `<name>/compressed` (`sensor_msgs/msg/CompressedImage`) when `publish_compressed=true`

### PoseController
- `<name>` (`nav_msgs/msg/Odometry`)

### ImuController
- `<device>/imu` (`sensor_msgs/msg/Imu`)

### LidarController
- `<name>_custom_msg` (`controller_msg/msg/CustomMsg`)
- `<name>_point_cloud` (`sensor_msgs/msg/PointCloud2`)
- Optional dynamic extrinsics subscriber:
	- `<name>/extrinsics` (`geometry_msgs/msg/PoseStamped`)

---

## 9) Hardware notes

### Intel T265
- Configured through `t265_camera_hardware` plugin.
- Typical interfaces exposed: `image0`, `image1`, `pose`, `gyro`, `accel`.
- Requires `librealsense2` and valid camera access permissions.

### Livox MID360
- Configured through `mid360_lidar_hardware` plugin.
- Network parameters are set in URDF/xacro hardware params.
- Supports optional dynamic extrinsics and IMU integration path in lidar controller.

### USB camera
- Configured through `usb_camera_hardware` plugin.
- Device path, format, and intrinsics are configured in xacro params.

---

## 10) Useful checks

```bash
# Controllers
ros2 control list_controllers

# Hardware interfaces
ros2 control list_hardware_interfaces

# Topic overview
ros2 topic list

# Example topic rate checks
ros2 topic hz /t265_front/imu
ros2 topic hz /mid360_front/lidar_point_cloud
```

---

## 11) Troubleshooting

### No sensor data is published
- Check controller state:
	- `ros2 control list_controllers`
- Verify interface names in `test_params.yaml` match hardware interface names.
- Confirm device permissions (`/dev/video*`, USB, network).

### T265 not detected
- Check cable/USB topology and camera serial.
- Verify `librealsense2` installation.

### MID360 no points
- Verify lidar/host IP and ports in xacro.
- Ensure firewall/network path allows UDP traffic.
- Check if lidar hardware plugin is loaded and active.

### High CPU usage or latency
- Keep CPU pinning (`taskset`) for deterministic scheduling.
- Reduce image compression/undistortion load where possible.
- Lower controller update rates if required by platform constraints.

---

## 12) Development tips

- Keep interface naming consistent across:
	1. URDF `state_interface`
	2. controller parameter lists
	3. controller code expectations
- Add new sensors by:
	1. creating/extending hardware interface plugin
	2. exposing `state_interface` entries
	3. adding controller mapping in `test_params.yaml`
	4. spawning controller in launch file

---

## 13) License

Licenses are defined per package (for example `controller_msg` contains a `LICENSE` file). Please review each package before redistribution.

