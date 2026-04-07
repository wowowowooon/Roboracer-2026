#!/usr/bin/env python3

from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from builtin_interfaces.msg import Time
from sensor_msgs.msg import LaserScan


class ScanRateAdapter(Node):
    def __init__(self) -> None:
        super().__init__("scan_rate_adapter")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_40hz")
        self.declare_parameter("target_hz", 40.0)
        self.declare_parameter("update_stamp", True)

        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_hz = float(self.get_parameter("target_hz").value)
        self.update_stamp = bool(self.get_parameter("update_stamp").value)

        if self.target_hz <= 0.0:
            raise ValueError("target_hz must be > 0")

        sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_scan: Optional[LaserScan] = None
        self.sub = self.create_subscription(
            LaserScan, self.input_topic, self._scan_cb, sensor_qos
        )
        self.pub = self.create_publisher(LaserScan, self.output_topic, sensor_qos)

        period = 1.0 / self.target_hz
        self.timer = self.create_timer(period, self._publish_latest)
        self.get_logger().info(
            f"Republishing {self.input_topic} -> {self.output_topic} at {self.target_hz:.1f} Hz"
        )

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg

    def _publish_latest(self) -> None:
        if self.latest_scan is None:
            return

        out = LaserScan()
        out.header = self.latest_scan.header
        if self.update_stamp:
            now = self.get_clock().now().to_msg()
            if isinstance(now, Time):
                out.header.stamp = now
        out.angle_min = self.latest_scan.angle_min
        out.angle_max = self.latest_scan.angle_max
        out.angle_increment = self.latest_scan.angle_increment
        out.scan_time = 1.0 / self.target_hz
        if len(self.latest_scan.ranges) > 0:
            out.time_increment = out.scan_time / float(len(self.latest_scan.ranges))
        else:
            out.time_increment = self.latest_scan.time_increment
        out.range_min = self.latest_scan.range_min
        out.range_max = self.latest_scan.range_max
        out.ranges = self.latest_scan.ranges
        out.intensities = self.latest_scan.intensities
        self.pub.publish(out)


def main() -> None:
    rclpy.init()
    node = ScanRateAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
