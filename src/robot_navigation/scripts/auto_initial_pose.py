#!/usr/bin/env python3
import math
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.parameter import Parameter


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class AutoInitialPose(Node):
    def __init__(self):
        super().__init__(
            'auto_initial_pose',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('map_offset_x', -1.45)
        self.declare_parameter('map_offset_y', -2.60)
        self.declare_parameter('map_offset_yaw', 0.0)
        self.declare_parameter('initial_delay_sec', 6.0)
        self.declare_parameter('publish_count', 30)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('initial_cov_xy', 0.25)
        self.declare_parameter('initial_cov_yaw', 0.07)

        self.odom = None
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

    def odom_cb(self, msg):
        self.odom = msg

    def wait_for_odom(self, timeout_sec=15.0):
        start = time.time()
        while rclpy.ok() and self.odom is None and time.time() - start < timeout_sec:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.odom is not None

    def build_initial_pose(self):
        map_offset_x = float(self.get_parameter('map_offset_x').value)
        map_offset_y = float(self.get_parameter('map_offset_y').value)
        map_offset_yaw = float(self.get_parameter('map_offset_yaw').value)
        cov_xy = float(self.get_parameter('initial_cov_xy').value)
        cov_yaw = float(self.get_parameter('initial_cov_yaw').value)

        odom_pose = self.odom.pose.pose
        odom_x = odom_pose.position.x
        odom_y = odom_pose.position.y
        odom_yaw = yaw_from_quat(odom_pose.orientation)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = map_offset_x + odom_x
        msg.pose.pose.position.y = map_offset_y + odom_y
        qx, qy, qz, qw = yaw_to_quat(map_offset_yaw + odom_yaw)
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0] = cov_xy
        msg.pose.covariance[7] = cov_xy
        msg.pose.covariance[35] = cov_yaw
        return msg

    def publish_initial_pose(self):
        delay = float(self.get_parameter('initial_delay_sec').value)
        count = int(self.get_parameter('publish_count').value)
        period = float(self.get_parameter('publish_period_sec').value)

        end = time.time() + delay
        while rclpy.ok() and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)

        if not self.wait_for_odom():
            self.get_logger().error('No /odom received, cannot publish automatic initial pose')
            return

        pose = self.build_initial_pose()
        self.get_logger().info(
            'auto initial pose: x=%.2f y=%.2f yaw=%.2f rad'
            % (
                pose.pose.pose.position.x,
                pose.pose.pose.position.y,
                yaw_from_quat(pose.pose.pose.orientation),
            )
        )

        for i in range(count):
            pose.header.stamp = self.get_clock().now().to_msg()
            self.initial_pose_pub.publish(pose)
            rclpy.spin_once(self, timeout_sec=0.05)
            time.sleep(period)

        self.get_logger().info('waiting for AMCL convergence (5s)...')
        converge_end = time.time() + 5.0
        while rclpy.ok() and time.time() < converge_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info('AMCL convergence wait complete, ready for navigation')


def main():
    rclpy.init()
    node = AutoInitialPose()
    try:
        node.publish_initial_pose()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
