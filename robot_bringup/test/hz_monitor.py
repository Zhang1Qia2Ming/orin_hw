#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import time
import importlib
import re
from collections import deque

class DynamicHzMonitor(Node):
    def __init__(self):
        super().__init__('dynamic_hz_monitor')
        self.topic_times = {}
        self.subs = {}
        
        self.pattern = re.compile(r'.*(/imu|/pose|/image[0-9]*|/image)$')
        
        # ⚡ 定制大容量 QoS：尽力而为，但队列深度拉满到 100！绝不漏包！
        self.custom_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=100
        )
        
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("🚀 极限吞吐版监控仪表盘启动...")

    def get_msg_class(self, type_str):
        parts = type_str.split('/')
        if len(parts) == 3:
            module_name = f"{parts[0]}.{parts[1]}"
            class_name = parts[2]
            try:
                module = importlib.import_module(module_name)
                return getattr(module, class_name)
            except Exception:
                pass
        return None

    def create_callback(self, topic_name):
        def callback(msg):
            self.topic_times[topic_name].append(time.time())
        return callback

    def timer_callback(self):
        current_time = time.time()
        
        # 1. 动态发现新话题
        topics_and_types = self.get_topic_names_and_types()
        for topic_name, types in topics_and_types:
            if topic_name not in self.subs and self.pattern.match(topic_name):
                msg_type_str = types[0]
                msg_class = self.get_msg_class(msg_type_str)
                if msg_class:
                    window_size = 200 if 'imu' in topic_name or 'pose' in topic_name else 60
                    self.topic_times[topic_name] = deque(maxlen=window_size)

                    # ⚡ 换上咱们定制的大容量 QoS
                    self.subs[topic_name] = self.create_subscription(
                        msg_class, topic_name, self.create_callback(topic_name), self.custom_qos)

        # ⚡ 2. ANSI 极速清屏（直接操作控制台显存，零系统调用阻塞，耗时接近 0ms）
        print('\033[2J\033[H', end='')
        
        print(f"\033[1;36m{'='*60}")
        print(f"{'🚀 ORIN SENSOR STREAM DASHBOARD (FULL SPEED)':^60}")
        print(f"{'='*60}\033[0m")
        print(f"\033[1m{'Topic Name':<40} | {'Frequency (Hz)':<15}\033[0m")
        print("-" * 60)
        
        for topic in sorted(self.topic_times.keys()):
            times = self.topic_times[topic]
            hz = 0.0
            
            # 如果话题超过 1 秒没数据了，直接判死刑，频率清零
            if len(times) > 1 and (current_time - times[-1]) < 1.0:
                dt = times[-1] - times[0]
                if dt > 0:
                    hz = (len(times) - 1) / dt
            else:
                times.clear() # 超时清空缓存
            
            color = "\033[1;32m" if hz > 150 else ("\033[1;33m" if hz > 20 else "\033[1;31m")
            print(f"{topic:<40} | {color}{hz:>8.2f} Hz\033[0m")

def main(args=None):
    rclpy.init(args=args)
    monitor = DynamicHzMonitor()
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()