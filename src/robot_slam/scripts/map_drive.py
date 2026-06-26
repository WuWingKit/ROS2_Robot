#!/usr/bin/env python3
"""
SLAM建图自动巡游脚本（开环里程计闭环控制）
机器人从入口出发，沿场景中轴线(y=2.5)穿过障碍物间隙缓慢巡游，
并在关键点原地旋转360°以让激光雷达充分扫描，生成完整地图。
速度保持低速，避免激光数据丢失。
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


def yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class Driver(Node):
    def __init__(self):
        super().__init__('map_drive')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.x = None
        self.y = None
        self.th = None

    def cb(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.th = yaw(m.pose.pose.orientation)

    def wait_odom(self):
        while self.x is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def cmd(self, lx, az):
        t = Twist()
        t.linear.x = lx
        t.angular.z = az
        self.pub.publish(t)

    def stop(self):
        for _ in range(6):
            self.cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.02)

    def spin360(self, rate=0.25):
        acc = 0.0
        prev = self.th
        while acc < 2 * math.pi:
            self.cmd(0.0, rate)
            rclpy.spin_once(self, timeout_sec=0.05)
            d = self.th - prev
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            acc += abs(d)
            prev = self.th
            time.sleep(0.02)
        self.stop()

    def goto(self, tx, ty, tol=0.15, tmax=70):
        t0 = time.time()
        while time.time() - t0 < tmax:
            rclpy.spin_once(self, timeout_sec=0.05)
            dx = tx - self.x
            dy = ty - self.y
            dist = math.hypot(dx, dy)
            if dist < tol:
                break
            target = math.atan2(dy, dx)
            err = target - self.th
            while err > math.pi:
                err -= 2 * math.pi
            while err < -math.pi:
                err += 2 * math.pi
            if abs(err) > 0.35:
                self.cmd(0.0, max(-0.3, min(0.3, 1.0 * err)))
            else:
                self.cmd(min(0.12, 0.3 * dist), max(-0.25, min(0.25, 0.8 * err)))
            time.sleep(0.02)
        self.stop()


def main():
    rclpy.init()
    d = Driver()
    d.wait_odom()
    # 沿中轴线穿过间隙巡游 + 关键点360°旋转
    seq = [
        ('spin',),
        ('go', 2.5, 2.5),
        ('spin',),
        ('go', 4.5, 2.5),
        ('spin',),
        ('go', 5.3, 2.5),
        ('spin',),
        ('go', 4.5, 2.5),
        ('go', 2.5, 2.5),
        ('go', 1.2, 2.5),
        ('spin',),
    ]
    for s in seq:
        if s[0] == 'spin':
            d.spin360()
        else:
            d.goto(s[1], s[2])
        print('  step %s -> pos(%.2f, %.2f)' % (str(s), d.x, d.y))
    d.stop()
    print('DRIVE COMPLETE at (%.2f, %.2f)' % (d.x, d.y))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
