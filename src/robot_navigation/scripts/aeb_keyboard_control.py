#!/usr/bin/env python3
import math
import os
import select
import subprocess
import sys
import termios
import threading
import time
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan


HELP = """
Keyboard control with AEB

  w : forward        x : backward
  a : turn left      d : turn right
  s/space : stop     q : quit

When the nearest /scan obstacle is closer than the AEB threshold,
the node publishes zero velocity and shows an "AEB triggered" popup.
"""


class AebKeyboardControl(Node):
    def __init__(self):
        super().__init__(
            'aeb_keyboard_control',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.declare_parameter('aeb_distance', 0.45)
        self.declare_parameter('release_distance', 0.60)
        self.declare_parameter('linear_speed', 0.18)
        self.declare_parameter('angular_speed', 0.65)

        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, sensor_qos)

        self.aeb_distance = float(self.get_parameter('aeb_distance').value)
        self.release_distance = float(self.get_parameter('release_distance').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.angular_speed = float(self.get_parameter('angular_speed').value)

        self.min_scan = math.inf
        self.aeb_active = False
        self.popup_open = False
        self.last_status_time = 0.0

    def scan_cb(self, msg):
        vals = [r for r in msg.ranges if msg.range_min < r < msg.range_max and math.isfinite(r)]
        self.min_scan = min(vals) if vals else math.inf
        if self.min_scan < self.aeb_distance and not self.aeb_active:
            self.aeb_active = True
            self.stop()
            self.get_logger().warn('AEB triggered: nearest obstacle %.2fm' % self.min_scan)
            self.show_popup()
        elif self.aeb_active and self.min_scan > self.release_distance:
            self.aeb_active = False
            self.get_logger().info('AEB released: nearest obstacle %.2fm' % self.min_scan)

    def stop(self):
        self.cmd_pub.publish(Twist())

    def publish_cmd(self, linear, angular):
        if self.aeb_active:
            self.stop()
            return
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self.cmd_pub.publish(msg)

    def show_popup(self):
        if self.popup_open:
            return
        self.popup_open = True

        def worker():
            text = 'AEB triggered! Obstacle too close. Robot stopped.'
            try:
                if os.environ.get('DISPLAY'):
                    if subprocess.call(['which', 'zenity'], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0:
                        subprocess.call(['zenity', '--warning', '--title=AEB triggered', '--text=' + text])
                    else:
                        import tkinter as tk
                        from tkinter import messagebox
                        root = tk.Tk()
                        root.withdraw()
                        messagebox.showwarning('AEB triggered', text)
                        root.destroy()
                else:
                    print('\n[AEB triggered] %s\n' % text)
            except Exception as exc:
                print('\n[AEB triggered] %s (%s)\n' % (text, exc))
            finally:
                self.popup_open = False

        threading.Thread(target=worker, daemon=True).start()

    def print_status(self):
        now = time.time()
        if now - self.last_status_time < 0.8:
            return
        self.last_status_time = now
        dist = 'inf' if math.isinf(self.min_scan) else '%.2fm' % self.min_scan
        state = 'ACTIVE' if self.aeb_active else 'ready'
        print('\rAEB: %-6s  nearest: %-6s  threshold: %.2fm   ' %
              (state, dist, self.aeb_distance), end='', flush=True)


def get_key(settings):
    tty.setraw(sys.stdin.fileno())
    ready, _, _ = select.select([sys.stdin], [], [], 0.08)
    key = sys.stdin.read(1) if ready else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def main():
    settings = termios.tcgetattr(sys.stdin)
    rclpy.init()
    node = AebKeyboardControl()
    print(HELP)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)
            key = get_key(settings)
            if key == 'q':
                break
            if key == 'w':
                node.publish_cmd(node.linear_speed, 0.0)
            elif key == 'x':
                node.publish_cmd(-node.linear_speed, 0.0)
            elif key == 'a':
                node.publish_cmd(0.0, node.angular_speed)
            elif key == 'd':
                node.publish_cmd(0.0, -node.angular_speed)
            elif key in ('s', ' '):
                node.stop()
            else:
                node.print_status()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print('\nAEB keyboard control stopped.')


if __name__ == '__main__':
    main()
