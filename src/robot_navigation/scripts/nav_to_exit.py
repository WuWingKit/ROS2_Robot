#!/usr/bin/env python3
"""
Nav2 closed-loop navigation demo and verifier.

The Gazebo world coordinates and the Cartographer map coordinates are not
identical, so this script uses the measured map-frame entrance/exit points.
It publishes the initial AMCL pose, sends a NavigateToPose action goal, then
prints a PASS/FAIL result suitable for demo recording or regression testing.
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose


# 地图由 Cartographer 构建, map 坐标系相对 Gazebo 世界系存在平移和轻微旋转。
# 经地图图像解析得到：入口缺口中心约 (-0.95, -0.1), 出口缺口中心约 (6.9, -0.1)。
# 机器人出生于 Gazebo(1.0, 2.5), 对应 map 系约 (-0.45, -0.1)。
INIT_GUESS = (-0.45, -0.1)   # map 系出生点(AMCL 初始猜测)
GOAL_ABS = (6.5, -0.1)       # map 系出口目标(略入内, 保证可达)
ARRIVE_TOL = 0.45    # 到达阈值(m): 目标点位于1.2m宽出口缺口内0.4m, 达此即视为抵达出口
COLLISION_DIST = 0.12  # 雷达min 0.1, 小于此视为碰撞


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class NavVerifier(Node):
    def __init__(self, start, goal, arrive_tol, collision_dist):
        # 必须使用仿真时间, 否则 initialpose 时间戳为墙钟时间, AMCL 无法变换(外推到未来)
        super().__init__('nav_to_exit',
                         parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.init_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.ac = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.start = start
        self.goal = goal
        self.arrive_tol = arrive_tol
        self.collision_dist = collision_dist
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
            if m < self.collision_dist:
                self.collided = True

    def set_initial_pose(self):
        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = float(self.start[0])
        p.pose.pose.position.y = float(self.start[1])
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
        self.get_logger().info('initial pose set at %s' % str(self.start))

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
        self.goal_pose_pub.publish(g.pose)
        self.get_logger().info('sending goal %s' % str(self.goal))
        return self.ac.send_goal_async(g)


def main():
    parser = argparse.ArgumentParser(description='Run the autonomous entrance-to-exit Nav2 demo.')
    parser.add_argument('--timeout', type=float, default=150.0, help='Navigation timeout in seconds.')
    parser.add_argument('--start-x', type=float, default=INIT_GUESS[0], help='Initial AMCL pose x in map frame.')
    parser.add_argument('--start-y', type=float, default=INIT_GUESS[1], help='Initial AMCL pose y in map frame.')
    parser.add_argument('--goal-x', type=float, default=GOAL_ABS[0], help='Goal pose x in map frame.')
    parser.add_argument('--goal-y', type=float, default=GOAL_ABS[1], help='Goal pose y in map frame.')
    parser.add_argument('--arrive-tol', type=float, default=ARRIVE_TOL, help='PASS distance threshold in meters.')
    parser.add_argument('--collision-dist', type=float, default=COLLISION_DIST, help='Minimum scan range treated as collision.')
    args = parser.parse_args()

    rclpy.init()
    n = NavVerifier(
        start=(args.start_x, args.start_y),
        goal=(args.goal_x, args.goal_y),
        arrive_tol=args.arrive_tol,
        collision_dist=args.collision_dist,
    )

    # 先让仿真时钟/scan/TF 就绪(用 sim time, 时钟需 /clock 填充)
    t0 = time.time()
    while time.time() - t0 < 6:
        rclpy.spin_once(n, timeout_sec=0.1)
    # 多次发布初始位姿确保 AMCL 接收
    n.set_initial_pose()
    # 让 AMCL 收敛
    t0 = time.time()
    while time.time() - t0 < 5:
        rclpy.spin_once(n, timeout_sec=0.1)
    p0 = n.cur if n.cur else n.start
    print('localized start=(%.2f,%.2f)  ->  goal=(%.2f,%.2f)' % (p0[0], p0[1], n.goal[0], n.goal[1]))
    n.send_goal()

    start = time.time()
    reached = False
    min_d = 99.0
    while time.time() - start < args.timeout:
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.cur:
            d = math.hypot(n.cur[0] - n.goal[0], n.cur[1] - n.goal[1])
            min_d = min(min_d, d)
            if d < n.arrive_tol:
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
