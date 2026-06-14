#!/usr/bin/env python3
"""
任务2 终端驱动与跟随监视器
- 自动向 turtle1 发布速度指令(圆形轨迹, 让 turtle2 跟随)
- 订阅 /turtle1/pose 和 /turtle2/pose, 打印两个海龟的相对位姿
- 配合 tf2_echo 输出完整的变换矩阵
"""
import math
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


class TurtleDriveAndEcho(Node):
    def __init__(self):
        super().__init__('turtle_driver')
        self.pub1 = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.sub1 = self.create_subscription(Pose, '/turtle1/pose', self.pose1_cb, 10)
        self.sub2 = self.create_subscription(Pose, '/turtle2/pose', self.pose2_cb, 10)
        self.pose1 = None
        self.pose2 = None
        self.start = time.time()

        self.get_logger().info('=' * 60)
        self.get_logger().info('  任务2: 小海龟跟随 + TF坐标变换')
        self.get_logger().info('  turtle1 (领航) → turtle2 (跟随)')
        self.get_logger().info('  键盘控制 turtle1 运动, turtle2 自动跟随')
        self.get_logger().info('=' * 60)

    def pose1_cb(self, msg):
        self.pose1 = (msg.x, msg.y, msg.theta)
        self._print_status()

    def pose2_cb(self, msg):
        self.pose2 = (msg.x, msg.y, msg.theta)
        self._print_status()

    def _print_status(self):
        if self.pose1 and self.pose2:
            dx = self.pose2[0] - self.pose1[0]
            dy = self.pose2[1] - self.pose1[1]
            dist = math.hypot(dx, dy)
            dtheta = self.pose2[2] - self.pose1[2]
            elapsed = time.time() - self.start
            self.get_logger().info(
                't=%.1fs | turtle1(%.2f,%.2f,%.1f°) turtle2(%.2f,%.2f,%.1f°) | '
                'Δx=%.2f Δy=%.2f dist=%.2f Δθ=%.1f° | turtle2跟随turtle1' %
                (elapsed,
                 self.pose1[0], self.pose1[1], math.degrees(self.pose1[2]),
                 self.pose2[0], self.pose2[1], math.degrees(self.pose2[2]),
                 dx, dy, dist, math.degrees(dtheta))
            )

    def drive_turtle1(self):
        """发布圆形运动指令让 turtle1 持续运动"""
        cmd = Twist()
        cmd.linear.x = 2.0
        cmd.angular.z = 1.0
        self.pub1.publish(cmd)


def main():
    rclpy.init()
    node = TurtleDriveAndEcho()
    node.create_timer(0.5, node.drive_turtle1)  # 自动驱动
    # 也保留键盘控制能力, 提示用户可在新终端运行 teleop_key
    node.get_logger().info('提示: 在新终端运行 ros2 run turtlesim turtle_teleop_key 可手动控制')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
