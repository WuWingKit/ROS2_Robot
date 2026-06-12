#!/usr/bin/env python3
"""
导航闭环验证/执行脚本
- 在入口(0.5, 2.5, 朝向+x)设置初始位姿
- 通过 Nav2 NavigateToPose action 发送出口目标(7.5, 2.5)
- 监听 /amcl_pose 判定是否到达目标(误差<阈值)
- 监听 /scan 监测是否发生碰撞(最近障碍距离过小)
- 输出 PASS/FAIL
用法: ros2 run robot_navigation nav_to_exit.py  (或 python3 nav_to_exit.py)
"""
import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose


# 地图由 Cartographer 构建, 其 map 坐标系与 Gazebo 世界系存在平移+轻微旋转。
# 经地图图像解析得到实际位置(map系)：入口缺口中心(-0.95,-0.1), 出口缺口中心(6.9,-0.1)。
# 机器人出生于 Gazebo(1.0,2.5)≈入口右侧0.5m → map(-0.45,-0.1)。
INIT_GUESS = (-0.45, -0.1)   # map 系出生点(AMCL 初始猜测)
GOAL_ABS = (6.5, -0.1)       # map 系出口目标(略入内, 保证可达)
ARRIVE_TOL = 0.35    # 到达阈值(m)
COLLISION_DIST = 0.12  # 雷达min 0.1, 小于此视为碰撞


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NavVerifier(Node):
    def __init__(self):
        super().__init__('nav_to_exit')
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.cur = None
        self.min_scan = 99.0
        self.collided = False

    def amcl_cb(self, msg):
        self.cur = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def scan_cb(self, msg):
        vals = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if vals:
            m = min(vals)
            self.min_scan = m
            if m < COLLISION_DIST:
                self.collided = True

    def set_initial_pose(self):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = float(INIT_GUESS[0])
        p.pose.pose.position.y = float(INIT_GUESS[1])
        qx, qy, qz, qw = yaw_to_quat(0.0)
        p.pose.pose.orientation.z = qz
        p.pose.pose.orientation.w = qw
        p.pose.covariance[0] = 0.25
        p.pose.covariance[7] = 0.25
        p.pose.covariance[35] = 0.07
        for _ in range(10):
            p.header.stamp = self.get_clock().now().to_msg()
            self.init_pub.publish(p)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)
        self.get_logger().info('initial pose set at %s' % str(START))

    def send_goal(self):
        self.ac.wait_for_server()
        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.header.stamp = self.get_clock().now().to_msg()
        g.pose.pose.position.x = float(self.goal[0])
        g.pose.pose.position.y = float(self.goal[1])
        qx, qy, qz, qw = yaw_to_quat(0.0)
        g.pose.pose.orientation.z = qz
        g.pose.pose.orientation.w = qw
        self.get_logger().info('sending goal %s' % str(self.goal))
        return self.ac.send_goal_async(g)


def main():
    rclpy.init()
    n = NavVerifier()
    n.goal = GOAL_ABS
    timeout = float(sys.argv[1]) if len(sys.argv) > 1 else 150.0

    # wait for amcl + scan
    t0 = time.time()
    while n.cur is None and time.time() - t0 < 20:
        rclpy.spin_once(n, timeout_sec=0.2)
    n.set_initial_pose()
    # 让 AMCL 收敛
    t0 = time.time()
    while time.time() - t0 < 4:
        rclpy.spin_once(n, timeout_sec=0.1)
    p0 = n.cur if n.cur else INIT_GUESS
    print('localized start=(%.2f,%.2f)  ->  goal=(%.2f,%.2f)' % (p0[0], p0[1], n.goal[0], n.goal[1]))
    n.send_goal()

    start = time.time()
    reached = False
    min_d = 99.0
    while time.time() - start < timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.cur:
            d = math.hypot(n.cur[0] - n.goal[0], n.cur[1] - n.goal[1])
            min_d = min(min_d, d)
            if d < ARRIVE_TOL:
                reached = True
                break
    dur = time.time() - start
    pos = n.cur
    print('=== NAV RESULT ===')
    print('final_pose=%s  dist_to_goal=%.2f  min_dist=%.2f  collided=%s  dur=%.1fs' % (
        ('(%.2f,%.2f)' % pos) if pos else 'None',
        math.hypot(pos[0] - n.goal[0], pos[1] - n.goal[1]) if pos else -1,
        min_d, n.collided, dur))
    print('RESULT: %s' % ('PASS' if (reached and not n.collided) else 'FAIL'))
    rclpy.shutdown()


if __name__ == '__main__':
    main()
