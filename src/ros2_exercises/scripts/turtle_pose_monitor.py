#!/usr/bin/env python3
"""
任务1 终端位姿监视器
- 自动向 turtle1 发布速度指令(圆形轨迹)
- 订阅 /turtle1/pose, 实时打印 x, y, theta
- 模拟 rqt_plot 在终端的数据显示
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class PoseMonitor(Node):
    def __init__(self):
        super().__init__('pose_monitor')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.pose_cb, 10)
        self.latest = None
        self.start = time.time()
        self.get_logger().info('=== 任务1: turtle1 位姿监视器已启动 ===')
        self.get_logger().info('实时位姿数据 (x, y, theta):')
        self.get_logger().info('{:>8} {:>8} {:>8}'.format('x', 'y', 'theta'))
        self.get_logger().info('-' * 30)

    def pose_cb(self, msg):
        self.latest = (msg.x, msg.y, msg.theta)
        elapsed = time.time() - self.start
        self.get_logger().info(
            't=%.1fs | x=%.2f y=%.2f θ=%.2f rad (%.1f°)' %
            (elapsed, msg.x, msg.y, msg.theta, math.degrees(msg.theta))
        )

    def drive(self):
        """发布圆形运动指令"""
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 1.5
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = PoseMonitor()
    # 定时器: 每0.2s发布速度 + 打印位姿
    node.create_timer(0.5, node.drive)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
