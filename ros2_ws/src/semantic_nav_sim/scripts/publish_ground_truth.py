#!/usr/bin/env python3
"""Omniscient semantic oracle for the simulation ablation, never the baseline.

Uses arena SDF geometry, not camera detections. Both oracle and future VLM
producers publish the same full-resolution 0–254 semantic cost-grid contract.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from nav2_msgs.msg import Costmap
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def build_grid(world_path, glass_cost=254, grass_cost=0):
    if not all(isinstance(v, int) and 0 <= v <= 254 for v in (glass_cost, grass_cost)):
        raise ValueError('Semantic costs must be integers in 0..254')
    world = ET.parse(world_path).getroot().find('world')
    msg = Costmap()
    msg.header.frame_id = 'map'
    msg.metadata.layer = 'ground_truth'
    msg.metadata.resolution = 0.05
    msg.metadata.size_x = msg.metadata.size_y = 250
    msg.metadata.origin.position.x = msg.metadata.origin.position.y = -6.25
    msg.metadata.origin.orientation.w = 1.0
    coords = -6.25 + (np.arange(250) + 0.5) * 0.05
    x, y = np.meshgrid(coords, coords)
    costs = np.full((250, 250), 255, dtype=np.uint8)
    for name, cost, geometry in (
            ('glass_wall', glass_cost, 'link/collision/geometry/box/size'),
            ('grass_patch', grass_cost, 'link/visual/geometry/plane/size')):
        model = world.find(f"model[@name='{name}']")
        if model is None:
            raise ValueError(f'Missing {name} in world')
        pose = [float(v) for v in model.findtext('pose').split()]
        size = [float(v) for v in model.findtext(geometry).split()]
        if pose[3] != 0 or pose[4] != 0:
            raise ValueError('Oracle expects planar benchmark models')
        dx, dy = x - pose[0], y - pose[1]
        c, s = math.cos(pose[5]), math.sin(pose[5])
        inside = ((np.abs(c * dx + s * dy) <= size[0] / 2 + 1e-9) &
                  (np.abs(-s * dx + c * dy) <= size[1] / 2 + 1e-9))
        costs[inside] = cost
    msg.data = costs.ravel().tolist()
    return msg


class GroundTruthPublisher(Node):
    def __init__(self):
        super().__init__('semantic_ground_truth')
        self.declare_parameter('world', str(Path(get_package_share_directory(
            'semantic_nav_sim')) / 'worlds/arena.sdf'))
        self.declare_parameter('glass_cost', 254)
        self.declare_parameter('grass_cost', 0)
        self.grid = build_grid(self.get_parameter('world').value,
                               self.get_parameter('glass_cost').value,
                               self.get_parameter('grass_cost').value)
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.publisher = self.create_publisher(Costmap, '/semantic_costmap', qos)
        self.timer = self.create_timer(0.5, self.publish)
        self.get_logger().info('Ground-truth oracle: SDF geometry, not camera perception')

    def publish(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid.metadata.update_time = self.grid.header.stamp
        self.publisher.publish(self.grid)


def main():
    rclpy.init()
    node = GroundTruthPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
