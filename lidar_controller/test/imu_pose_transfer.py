#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from rclpy.qos import qos_profile_sensor_data
import math

class Mid360AttitudeEstimator(Node):
    def __init__(self):
        super().__init__('mid360_attitude_estimator')
        
        # 订阅原始 IMU (必须使用 SensorDataQoS，与底层硬件匹配)
        self.imu_sub = self.create_subscription(
            Imu,
            '/mid360_front/imu',
            self.imu_callback,
            qos_profile_sensor_data
        )
        
        # 发布给 lidar_controller 的动态外参
        self.pose_pub = self.create_publisher(
            PoseStamped,
            '/mid360_front/lidar/extrinsics',
            10
        )
        
        self.initialized = False
        self.last_time = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
        self.alpha = 0.98  # 融合权重，雷达抖动剧烈可以适当调大(如0.99)
        
        self.get_logger().info("🚀 Mid360 Attitude Estimator (Python 快速验证版) 已启动！")

    def euler_to_quaternion(self, r, p, y):
        """内部实现 RPY 转四元数，彻底摆脱 tf 库依赖"""
        cy = math.cos(y * 0.5)
        sy = math.sin(y * 0.5)
        cp = math.cos(p * 0.5)
        sp = math.sin(p * 0.5)
        cr = math.cos(r * 0.5)
        sr = math.sin(r * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def imu_callback(self, msg):
        # 提取时间戳 (秒)
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if not self.initialized:
            self.last_time = current_time
            # 初始对齐：完全信任第一帧加速度计提取的重力向量
            self.roll = math.atan2(-msg.linear_acceleration.y, msg.linear_acceleration.z)
            self.pitch = math.atan2(
                msg.linear_acceleration.x, 
                math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)
            )
            self.yaw = 0.0
            self.initialized = True
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        # 1. 加速度计观测 (限制分母防止除零)
        acc_roll = math.atan2(-msg.linear_acceleration.y, msg.linear_acceleration.z)
        acc_pitch = math.atan2(
            msg.linear_acceleration.x, 
            math.sqrt(msg.linear_acceleration.y**2 + msg.linear_acceleration.z**2)
        )

        # 2. 陀螺仪积分
        gyro_roll = self.roll + msg.angular_velocity.x * dt
        gyro_pitch = self.pitch + msg.angular_velocity.y * dt
        self.yaw = self.yaw + msg.angular_velocity.z * dt

        # 3. 互补滤波融合
        self.roll = self.alpha * gyro_roll + (1.0 - self.alpha) * acc_roll
        self.pitch = self.alpha * gyro_pitch + (1.0 - self.alpha) * acc_pitch

        # 4. 构建反向补偿 Pose (-roll, -pitch)
        # 注意：这里强行将 Yaw 对齐抵消设为 0.0，防止点云整体在水平面内乱转
        qx, qy, qz, qw = self.euler_to_quaternion(-self.roll, -self.pitch, 0.0)

        extrinsics_msg = PoseStamped()
        extrinsics_msg.header.stamp = msg.header.stamp
        extrinsics_msg.header.frame_id = "mid360_base_link"  # 根据你的 TF 树名称调整

        extrinsics_msg.pose.orientation.x = qx
        extrinsics_msg.pose.orientation.y = qy
        extrinsics_msg.pose.orientation.z = qz
        extrinsics_msg.pose.orientation.w = qw
        
        # 纯姿态补偿，平移保持 0
        extrinsics_msg.pose.position.x = 0.0
        extrinsics_msg.pose.position.y = 0.0
        extrinsics_msg.pose.position.z = 0.0

        self.pose_pub.publish(extrinsics_msg)

def main(args=None):
    rclpy.init(args=args)
    node = Mid360AttitudeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 手动停止...")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()