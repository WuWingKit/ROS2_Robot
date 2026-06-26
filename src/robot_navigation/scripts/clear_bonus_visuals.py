#!/usr/bin/env python3
import time

import rclpy
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray


class ClearBonusVisuals(Node):
    def __init__(self):
        super().__init__(
            'clear_bonus_visuals',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.path_history_pub = self.create_publisher(Path, '/bonus/path_history', 10)
        self.dijkstra_pub = self.create_publisher(Path, '/bonus/path_dijkstra', 10)
        self.astar_pub = self.create_publisher(Path, '/bonus/path_astar', 10)
        self.plan_pub = self.create_publisher(Path, '/plan', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        self.nav_markers_pub = self.create_publisher(MarkerArray, '/bonus/navigation_markers', 10)
        self.compare_markers_pub = self.create_publisher(MarkerArray, '/bonus/planner_compare_markers', 10)

    def empty_path(self):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        return msg

    def delete_all_markers(self):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(marker)
        return arr

    def clear(self):
        for _ in range(8):
            self.path_history_pub.publish(self.empty_path())
            self.dijkstra_pub.publish(self.empty_path())
            self.astar_pub.publish(self.empty_path())
            self.plan_pub.publish(self.empty_path())
            self.local_plan_pub.publish(self.empty_path())
            self.nav_markers_pub.publish(self.delete_all_markers())
            self.compare_markers_pub.publish(self.delete_all_markers())
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(0.1)


def main():
    rclpy.init()
    node = ClearBonusVisuals()
    try:
        node.clear()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
