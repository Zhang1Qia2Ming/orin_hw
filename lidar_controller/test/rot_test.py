#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math

class ContinuousRotationTester(Node):
    def __init__(self):
        super().__init__('lidar_rotation_tester')
        
        # 🌟 严格对应你的订阅话题
        self.publisher_ = self.create_publisher(PoseStamped, '/mid360_front/lidar/extrinsics', 10)
        
        # 10Hz 频率 (0.1秒执行一次)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        # 初始状态设定
        self.current_yaw_rad = 0.0 
        self.delta_yaw_rad = math.radians(2.0)  # 每次转 2 度 (10Hz下就是每秒转 20 度)
        
        self.get_logger().info("🔥 动态外参测试节点已启动，准备让点云转起来！")

    def euler_to_quaternion(self, roll, pitch, yaw):
        """ 极其干净的欧拉角转四元数纯数学实现，彻底摆脱外部依赖 """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy

        return qx, qy, qz, qw

    def timer_callback(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 假设参考系为机器人的躯干 base_link
        msg.header.frame_id = 'base_link'

        # 保持在脖子上方 15cm 处
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = 0.15

        # 将不断增加的 Yaw 角转换为底层需要的四元数
        qx, qy, qz, qw = self.euler_to_quaternion(0.0, 0.0, self.current_yaw_rad)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw

        self.publisher_.publish(msg)

        # 角度累加，并在超过 360 度 (2*Pi) 时重置，防止浮点数溢出
        self.current_yaw_rad += self.delta_yaw_rad
        if self.current_yaw_rad > 2 * math.pi:
            self.current_yaw_rad -= 2 * math.pi

def main(args=None):
    rclpy.init(args=args)
    node = ContinuousRotationTester()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()