#!/usr/bin/env python3
"""Publish a saved ROS map yaml on /map for RViz (latched + periodic refresh)."""

from __future__ import annotations

import os

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import OccupancyGrid
from PIL import Image
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


def _load_map_yaml(yaml_path: str) -> OccupancyGrid:
    with open(yaml_path, 'r', encoding='utf-8') as f:
        meta = yaml.safe_load(f) or {}

    image_path = meta.get('image', '')
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(yaml_path), image_path)
    if not os.path.isfile(image_path):
        stem, _ = os.path.splitext(image_path)
        for alt_ext in ('.pgm', '.png'):
            alt_path = stem + alt_ext
            if os.path.isfile(alt_path):
                image_path = alt_path
                break
    if not os.path.isfile(image_path):
        raise FileNotFoundError(f'map image not found: {image_path}')

    resolution = float(meta.get('resolution', 0.05))
    origin = meta.get('origin', [0.0, 0.0, 0.0])
    negate = int(meta.get('negate', 0))
    occupied_thresh = float(meta.get('occupied_thresh', 0.65))
    free_thresh = float(meta.get('free_thresh', 0.196))

    img = np.array(Image.open(image_path).convert('L'), dtype=np.float32)
    if negate:
        occ_prob = img / 255.0
    else:
        occ_prob = 1.0 - (img / 255.0)

    grid = np.full(img.shape, -1, dtype=np.int8)
    grid[occ_prob > occupied_thresh] = 100
    grid[occ_prob < free_thresh] = 0

    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width = int(img.shape[1])
    msg.info.height = int(img.shape[0])
    msg.info.origin.position.x = float(origin[0])
    msg.info.origin.position.y = float(origin[1])
    msg.info.origin.position.z = float(origin[2] if len(origin) > 2 else 0.0)
    msg.data = grid[::-1, :].reshape(-1).tolist()
    return msg


class StaticMapPublisher(Node):
    def __init__(self):
        super().__init__('static_map_publisher')
        self.declare_parameter('yaml_filename', '')
        self.declare_parameter('topic', '/map')
        self.declare_parameter('publish_period_sec', 1.0)

        yaml_path = self.get_parameter('yaml_filename').get_parameter_value().string_value
        topic = self.get_parameter('topic').get_parameter_value().string_value
        period = float(self.get_parameter('publish_period_sec').value)

        if not yaml_path or not os.path.isfile(yaml_path):
            raise RuntimeError(f'map yaml not found: {yaml_path}')

        # Latched map: RViz can subscribe late and still receive the grid.
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._pub = self.create_publisher(OccupancyGrid, topic, qos)
        self._msg = _load_map_yaml(yaml_path)
        free = sum(1 for v in self._msg.data if v == 0)
        occ = sum(1 for v in self._msg.data if v == 100)
        self.get_logger().info(
            f'Map -> {topic}: {self._msg.info.width}x{self._msg.info.height} '
            f'cells, res={self._msg.info.resolution:.3f}, free={free}, occ={occ}'
        )
        self._publish()
        if period > 0.0:
            self._timer = self.create_timer(period, self._publish)

    def _publish(self):
        self._msg.header.stamp = self.get_clock().now().to_msg()
        self._pub.publish(self._msg)


def main(args=None):
    rclpy.init(args=args)
    node = StaticMapPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
