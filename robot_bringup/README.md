
# robot_bringup

`robot_bringup` is the ROS 2 bringup package for the perception system based on `ros2_control`. It assembles the robot description (URDF/Xacro), loads controller configurations, and launches the `ros2_control_node` along with optional RViz and controller spawners.

## What this package provides

*   **Launch files** to start the control stack and optional visualization.
*   **Controller configuration** YAMLs for image, pose, IMU, and LiDAR controllers.
*   **Robot description** files (URDF/Xacro) and RViz configuration.

## Dependencies

Runtime dependencies are declared in [package.xml](src/orin_Hw/robot_bringup/package.xml):
*   `controller_manager`
*   `ros2_control`
*   `launch`, `launch_ros`
*   `rclcpp`

## Package layout

*   **launch/**
	*   `test.launch.py` — Full bringup: `ros2_control_node`, state publisher, controller spawners, RViz.
	*   `test_lower_half.launch.py` — Minimal bringup: `ros2_control_node` + state publisher (no controllers, no RViz).
	*   `test_rviz.launch.py` — Full bringup with RViz.
*   **config/**
	*   `test_params.yaml` — Controller manager and controller parameters for image/pose/IMU/LiDAR.
	*   `test_none.yaml` — Minimal controller manager config.
	*   `MID360_config.json` — Example LiDAR SDK config.
*   **urdf/**
	*   `robot.xacro` — Top-level robot description.
	*   `robot.urdf.xacro`, `robot.ros2_control.xacro`, `robot.materials.xacro` — Components included by `robot.xacro`.
*   **rviz/**
	*   `robot.rviz` — Default RViz configuration.

## Launch usage

Examples (from your workspace root):

*   Full bringup with controllers + RViz:
	```bash
	ros2 launch robot_bringup test.launch.py
	```

*   Full bringup with RViz (explicit):
	```bash
	ros2 launch robot_bringup test_rviz.launch.py
	```

*   Minimal bringup (control node + state publisher only):
	```bash
	ros2 launch robot_bringup test_lower_half.launch.py
	```

## Key launch arguments

All launch files accept:
*   `prefix` — Joint name prefix.
*   `use_mock_hardware` — Use mock hardware for ros2_control.
*   `mock_sensor_commands` — Enable mock sensor commands.
*   `slowdown` — Simulation slowdown factor.

## Notes

*   `test_params.yaml` wires the controller manager to the image/pose/IMU/LiDAR controllers defined in this workspace.
*   The `robot.xacro` file is the entry point for the robot description; it includes the `ros2_control` configuration and materials.
