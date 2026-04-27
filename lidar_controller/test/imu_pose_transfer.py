#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data

class ImuPoseTransfer(Node):
    def __init__(self):
        super().__init__('imu_pose_transfer')

        # Subscribe IMU raw data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            qos_profile_sensor_data
        )

        # Publish PoseStamped extrinsics
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mid360_front/lidar/extrinsics',
            10
        )

        self.get_logger().info('IMU to PoseStamped transfer node started.')

    def imu_callback(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header = msg.header

        # Keep source frame id if available, otherwise set a default frame.
        if not pose_msg.header.frame_id:
            pose_msg.header.frame_id = 'livox_frame'

        pose_msg.pose.orientation = msg.orientation

        # Position is not provided by IMU; keep zero translation.
        pose_msg.pose.position.x = 0.0
        pose_msg.pose.position.y = 0.0
        pose_msg.pose.position.z = 0.0

        self.pose_pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ImuPoseTransfer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Stopped by user.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()