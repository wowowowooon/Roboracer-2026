#!/usr/bin/env python3
"""Publish coherent dummy telemetry for CSV logger testing."""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class TelemetryDummyPublisher(Node):
    def __init__(self) -> None:
        super().__init__("telemetry_dummy_publisher")
        self.declare_parameter("publish_hz", 20.0)
        hz = max(1.0, float(self.get_parameter("publish_hz").value))
        self._start = time.monotonic()
        self._vesc_pub = self.create_publisher(
            Float64MultiArray, "/vesc/status", 10
        )
        self._rc_pub = self.create_publisher(
            Float64MultiArray, "/rc/state", 10
        )
        self._control_pub = self.create_publisher(
            Float64MultiArray, "/control/state", 10
        )
        self._safety_pub = self.create_publisher(
            Float64MultiArray, "/safety/state", 10
        )
        self.create_timer(1.0 / hz, self._publish)
        self.get_logger().info(f"Dummy telemetry started ({hz:.1f} Hz)")

    @staticmethod
    def _message(values) -> Float64MultiArray:
        msg = Float64MultiArray()
        msg.data = [float(value) for value in values]
        return msg

    def _publish(self) -> None:
        elapsed = time.monotonic() - self._start
        stamp = self.get_clock().now().nanoseconds * 1e-9
        erpm = -4200.0 - 300.0 * math.sin(elapsed)
        duty = -0.08 - 0.01 * math.sin(elapsed)
        voltage = 14.1 - 0.05 * math.sin(elapsed * 0.5)

        self._vesc_pub.publish(
            self._message(
                [stamp, voltage, 3.2, 8.5, duty, erpm, 42.0, 48.0, 0.0]
            )
        )
        # Current CH5 intent is preserved: low=auto, high=manual.
        self._rc_pub.publish(
            self._message(
                [stamp, 1500.0, 1600.0, 1000.0, 1000.0, 0.0, 1.0, 0.0]
            )
        )
        self._control_pub.publish(
            self._message([stamp, duty, 0.05 * math.sin(elapsed)])
        )
        self._safety_pub.publish(
            self._message(
                [stamp, 0.0, 0.0, 0.0, 0.0, duty, duty, duty, 0.0, 0.0]
            )
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TelemetryDummyPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
