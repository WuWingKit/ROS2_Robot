#!/usr/bin/env python3
import argparse
import heapq
import math
import os
import time
from collections import deque

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.parameter import Parameter
from visualization_msgs.msg import Marker, MarkerArray


START = (-0.45, -0.10)
GOAL = (6.50, -0.10)


def read_map_yaml(path):
    data = {}
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            if ':' not in line:
                continue
            key, value = line.split(':', 1)
            data[key.strip()] = value.strip()
    origin = data.get('origin', '[0, 0, 0]').strip('[]')
    origin_vals = [float(v.strip()) for v in origin.split(',')]
    image = data['image']
    if not os.path.isabs(image):
        image = os.path.join(os.path.dirname(path), image)
    return image, float(data['resolution']), origin_vals


def read_pgm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        if magic != b'P5':
            raise ValueError('Only binary PGM (P5) maps are supported')
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        width, height = [int(x) for x in line.split()]
        max_value = int(f.readline().strip())
        pixels = list(f.read(width * height))
    if max_value <= 0:
        raise ValueError('Invalid PGM max value')
    return width, height, pixels


def inflate_obstacles(free, width, height, radius_cells):
    if radius_cells <= 0:
        return free
    inflated = free[:]
    occupied = [(i % width, i // width) for i, ok in enumerate(free) if not ok]
    offsets = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy <= radius_cells * radius_cells:
                offsets.append((dx, dy))
    for x, y in occupied:
        for dx, dy in offsets:
            nx, ny = x + dx, y + dy
            if 0 <= nx < width and 0 <= ny < height:
                inflated[ny * width + nx] = False
    return inflated


class GridPlanner:
    def __init__(self, map_yaml, inflate_radius):
        image, self.resolution, self.origin = read_map_yaml(map_yaml)
        self.width, self.height, pixels = read_pgm(image)
        # In ROS maps, high pixel values are free, low values are occupied.
        free = [p > 220 for p in pixels]
        radius_cells = int(math.ceil(inflate_radius / self.resolution))
        self.free = inflate_obstacles(free, self.width, self.height, radius_cells)

    def world_to_grid(self, x, y):
        gx = int(round((x - self.origin[0]) / self.resolution))
        gy_from_bottom = int(round((y - self.origin[1]) / self.resolution))
        gy = self.height - 1 - gy_from_bottom
        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.origin[0] + gx * self.resolution
        y = self.origin[1] + (self.height - 1 - gy) * self.resolution
        return x, y

    def is_free(self, node):
        x, y = node
        return 0 <= x < self.width and 0 <= y < self.height and self.free[y * self.width + x]

    def nearest_free(self, node, limit=30):
        if self.is_free(node):
            return node
        q = deque([node])
        seen = {node}
        while q:
            x, y = q.popleft()
            if abs(x - node[0]) > limit or abs(y - node[1]) > limit:
                continue
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (x + dx, y + dy)
                if nxt in seen:
                    continue
                seen.add(nxt)
                if self.is_free(nxt):
                    return nxt
                q.append(nxt)
        raise RuntimeError('No nearby free cell found for %s' % (node,))

    def neighbors(self, node):
        x, y = node
        for dx, dy, cost in (
            (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
            (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2))
        ):
            nxt = (x + dx, y + dy)
            if self.is_free(nxt):
                yield nxt, cost * self.resolution

    def plan(self, start, goal, heuristic):
        start = self.nearest_free(self.world_to_grid(*start))
        goal = self.nearest_free(self.world_to_grid(*goal))
        open_heap = [(0.0, start)]
        came_from = {}
        cost_so_far = {start: 0.0}
        visited = 0
        t0 = time.perf_counter()

        while open_heap:
            _, current = heapq.heappop(open_heap)
            visited += 1
            if current == goal:
                break
            for nxt, step_cost in self.neighbors(current):
                new_cost = cost_so_far[current] + step_cost
                if nxt not in cost_so_far or new_cost < cost_so_far[nxt]:
                    cost_so_far[nxt] = new_cost
                    gx, gy = goal
                    nx, ny = nxt
                    priority = new_cost + heuristic * math.hypot(gx - nx, gy - ny) * self.resolution
                    heapq.heappush(open_heap, (priority, nxt))
                    came_from[nxt] = current

        elapsed_ms = (time.perf_counter() - t0) * 1000.0
        if goal not in cost_so_far:
            raise RuntimeError('Planning failed')

        cells = [goal]
        while cells[-1] != start:
            cells.append(came_from[cells[-1]])
        cells.reverse()
        points = [self.grid_to_world(x, y) for x, y in cells]
        length = sum(math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
                     for i in range(1, len(points)))
        return {
            'points': points,
            'length': length,
            'visited': visited,
            'elapsed_ms': elapsed_ms,
        }


class PlannerComparePublisher(Node):
    def __init__(self, dijkstra, astar):
        super().__init__(
            'planner_compare',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)]
        )
        self.dijkstra = dijkstra
        self.astar = astar
        self.dijkstra_pub = self.create_publisher(Path, '/bonus/path_dijkstra', 10)
        self.astar_pub = self.create_publisher(Path, '/bonus/path_astar', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/bonus/planner_compare_markers', 10)
        self.timer = self.create_timer(1.0, self.publish)
        self.clear_previous_visuals()

    def clear_previous_visuals(self):
        empty = Path()
        empty.header.frame_id = 'map'
        delete_marker = Marker()
        delete_marker.header.frame_id = 'map'
        delete_marker.action = Marker.DELETEALL
        arr = MarkerArray()
        arr.markers.append(delete_marker)
        for _ in range(5):
            empty.header.stamp = self.get_clock().now().to_msg()
            delete_marker.header.stamp = self.get_clock().now().to_msg()
            self.dijkstra_pub.publish(empty)
            self.astar_pub.publish(empty)
            self.marker_pub.publish(arr)
            rclpy.spin_once(self, timeout_sec=0.05)

    def to_path(self, result):
        msg = Path()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        for x, y in result['points']:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            msg.poses.append(pose)
        return msg

    def publish(self):
        self.dijkstra_pub.publish(self.to_path(self.dijkstra))
        self.astar_pub.publish(self.to_path(self.astar))
        markers = MarkerArray()
        text = (
            'Planner comparison\n'
            'Dijkstra: %.2fm, %d cells, %.1fms\n'
            'A*: %.2fm, %d cells, %.1fms'
        ) % (
            self.dijkstra['length'], self.dijkstra['visited'], self.dijkstra['elapsed_ms'],
            self.astar['length'], self.astar['visited'], self.astar['elapsed_ms'],
        )
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'planner_compare'
        m.id = 1
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = 2.5
        m.pose.position.y = 1.15
        m.pose.position.z = 0.55
        m.pose.orientation.w = 1.0
        m.scale.z = 0.18
        m.color.r = 1.0
        m.color.g = 1.0
        m.color.b = 1.0
        m.color.a = 1.0
        m.text = text
        markers.markers.append(m)
        self.marker_pub.publish(markers)


def main():
    parser = argparse.ArgumentParser(description='Compare Dijkstra and A* on the saved navigation map.')
    default_map = os.path.join(get_package_share_directory('robot_navigation'), 'maps', 'map.yaml')
    parser.add_argument('--map', default=default_map)
    parser.add_argument('--start-x', type=float, default=START[0])
    parser.add_argument('--start-y', type=float, default=START[1])
    parser.add_argument('--goal-x', type=float, default=GOAL[0])
    parser.add_argument('--goal-y', type=float, default=GOAL[1])
    parser.add_argument('--inflate-radius', type=float, default=0.18)
    parser.add_argument('--publish', action='store_true', help='Keep publishing comparison paths for RViz')
    args = parser.parse_args()

    planner = GridPlanner(args.map, args.inflate_radius)
    start = (args.start_x, args.start_y)
    goal = (args.goal_x, args.goal_y)
    dijkstra = planner.plan(start, goal, heuristic=0.0)
    astar = planner.plan(start, goal, heuristic=1.0)

    print('=== Planner comparison ===')
    print('Dijkstra length=%.2fm visited=%d elapsed=%.1fms points=%d' % (
        dijkstra['length'], dijkstra['visited'], dijkstra['elapsed_ms'], len(dijkstra['points'])))
    print('A*       length=%.2fm visited=%d elapsed=%.1fms points=%d' % (
        astar['length'], astar['visited'], astar['elapsed_ms'], len(astar['points'])))

    if args.publish:
        rclpy.init()
        node = PlannerComparePublisher(dijkstra, astar)
        try:
            rclpy.spin(node)
        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
