#!/usr/bin/env python3
import math

import rclpy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray


START = (-0.45, -0.10)
GOAL = (6.50, -0.10)

OBSTACLES = [
    ('cube_1', 1.05, -1.10, 0.60, 0.60, Marker.CUBE),
    ('cube_2', 1.05, 0.90, 0.60, 0.60, Marker.CUBE),
    ('box_3', 3.05, -0.60, 0.80, 0.40, Marker.CUBE),
    ('box_4', 3.05, 0.40, 0.80, 0.40, Marker.CUBE),
    ('cylinder_5', 4.55, -0.10, 0.50, 0.50, Marker.CYLINDER),
]


class NavVisualizer(Node):
    def __init__(self):
        super().__init__(
            'nav_visualizer',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.path_pub = self.create_publisher(Path, '/bonus/path_history', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bonus/navigation_markers', 10)
        self.path = Path()
        self.path.header.frame_id = 'map'
        self.pose = None
        self.min_scan = None
        self.recording = False

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_pose_cb, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.pose_cb, 10)
        self.timer = self.create_timer(0.5, self.publish_visuals)
        self.clear_previous_visuals()

    def clear_previous_visuals(self):
        empty_path = Path()
        empty_path.header.frame_id = 'map'
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        delete_array = MarkerArray()
        delete_array.markers.append(delete_marker)
        for _ in range(5):
            empty_path.header.stamp = self.get_clock().now().to_msg()
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty_path)
            self.marker_pub.publish(delete_array)
            rclpy.spin_once(self, timeout_sec=0.05)

    def goal_cb(self, msg):
        global GOAL
        GOAL = (msg.pose.position.x, msg.pose.position.y)
        self.path.poses = []
        self.recording = True
        self.clear_previous_visuals()

    def initial_pose_cb(self, _msg):
        self.path.poses = []
        self.recording = False
        self.clear_previous_visuals()

    def scan_cb(self, msg):
        vals = [r for r in msg.ranges if msg.range_min < r < msg.range_max and math.isfinite(r)]
        if vals:
            self.min_scan = min(vals)

    def pose_cb(self, msg):
        self.pose = msg.pose.pose
        if not self.recording:
            return
        ps = PoseStamped()
        ps.header.frame_id = 'map'
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = msg.pose.pose
        if not self.path.poses:
            self.path.poses.append(ps)
        else:
            last = self.path.poses[-1].pose.position
            cur = ps.pose.position
            if math.hypot(cur.x - last.x, cur.y - last.y) > 0.03:
                self.path.poses.append(ps)
        self.path.poses = self.path.poses[-600:]

    def marker(self, marker_id, ns, marker_type, x, y, z, sx, sy, sz, color):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = marker_type
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = z
        m.pose.orientation.w = 1.0
        m.scale.x = sx
        m.scale.y = sy
        m.scale.z = sz
        m.color.r, m.color.g, m.color.b, m.color.a = color
        return m

    def text_marker(self, marker_id, text, x, y, z, size=0.18):
        m = self.marker(marker_id, 'bonus_text', Marker.TEXT_VIEW_FACING, x, y, z, 0.0, 0.0, size,
                        (1.0, 1.0, 1.0, 1.0))
        m.text = text
        return m

    def publish_visuals(self):
        markers = MarkerArray()
        markers.markers.append(self.marker(1, 'start_goal', Marker.SPHERE, START[0], START[1], 0.08,
                                           0.22, 0.22, 0.22, (0.0, 0.9, 0.2, 0.9)))
        markers.markers.append(self.text_marker(2, 'START', START[0], START[1] + 0.28, 0.35))
        markers.markers.append(self.marker(3, 'start_goal', Marker.SPHERE, GOAL[0], GOAL[1], 0.08,
                                           0.24, 0.24, 0.24, (1.0, 0.1, 0.1, 0.9)))
        markers.markers.append(self.text_marker(4, 'GOAL', GOAL[0], GOAL[1] + 0.28, 0.35))

        for idx, (name, x, y, sx, sy, shape) in enumerate(OBSTACLES, start=10):
            markers.markers.append(self.marker(idx, 'obstacle_bounds', shape, x, y, 0.05,
                                               sx, sy, 0.10, (1.0, 0.85, 0.0, 0.35)))
            markers.markers.append(self.text_marker(idx + 100, name, x, y, 0.38, 0.14))

        if self.pose:
            px = self.pose.position.x
            py = self.pose.position.y
            dist = math.hypot(px - GOAL[0], py - GOAL[1])
            scan_txt = 'min scan: %.2fm' % self.min_scan if self.min_scan else 'min scan: --'
            markers.markers.append(self.text_marker(
                200,
                'distance to goal: %.2fm\n%s\npath samples: %d' % (dist, scan_txt, len(self.path.poses)),
                px,
                py + 0.45,
                0.55,
                0.16
            ))

        self.path.header.stamp = self.get_clock().now().to_msg()
        if self.recording:
            self.path_pub.publish(self.path)
        else:
            empty_path = Path()
            empty_path.header.frame_id = 'map'
            empty_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(empty_path)
        self.marker_pub.publish(markers)


def main():
    rclpy.init()
    node = NavVisualizer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
