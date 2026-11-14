#!/usr/bin/env python3
"""Convert raw VESC status into vehicle-frame measurements."""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class VehicleMeasurementNode(Node):
    """Publish speed and acceleration derived from `/vesc/status`."""

    def __init__(self) -> None:
        super().__init__("vehicle_measurement_node")
        # These defaults intentionally match control_node's working conversion.
        self.declare_parameter("pole_pairs", 2.0)
        self.declare_parameter("gear_ratio", 12.0)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("nominal_voltage", 14.4)
        self.declare_parameter("invert_speed_sign", True)
        self.declare_parameter("max_abs_accel_mps2", 30.0)

        self._pole_pairs = max(
            1e-6, float(self.get_parameter("pole_pairs").value)
        )
        self._gear_ratio = max(
            1e-6, float(self.get_parameter("gear_ratio").value)
        )
        self._wheel_radius = max(
            1e-6, float(self.get_parameter("wheel_radius").value)
        )
        self._nominal_voltage = float(
            self.get_parameter("nominal_voltage").value
        )
        self._invert_speed_sign = bool(
            self.get_parameter("invert_speed_sign").value
        )
        self._max_abs_accel = max(
            0.0, float(self.get_parameter("max_abs_accel_mps2").value)
        )
        self._last_speed = None
        self._last_stamp = None
        self._last_accel = 0.0

        self._publisher = self.create_publisher(
            Float64MultiArray, "/vehicle/state", 10
        )
        self.create_subscription(
            Float64MultiArray, "/vesc/status", self._on_vesc_status, 10
        )
        self.get_logger().info("Vehicle measurement node started")

    @staticmethod
    def _value(data, index: int, default=math.nan) -> float:
        if index >= len(data):
            return default
        value = float(data[index])
        return value if math.isfinite(value) else default

    def _on_vesc_status(self, msg: Float64MultiArray) -> None:
        # VESC layout: [stamp, voltage, input_current, motor_current,
        # duty_now, erpm, vesc_temp, motor_temp, fault_code]
        stamp = self._value(msg.data, 0)
        voltage = self._value(msg.data, 1)
        erpm = self._value(msg.data, 5)
        if not math.isfinite(stamp) or not math.isfinite(erpm):
            return

        # Keep conversion aligned with control_node's working speed formula.
        motor_rpm = erpm / self._pole_pairs
        wheel_rpm = motor_rpm / self._gear_ratio
        wheel_circumference = 2.0 * math.pi * self._wheel_radius
        speed_mps = wheel_rpm * wheel_circumference / 60.0
        if self._invert_speed_sign:
            speed_mps = -speed_mps
        speed_kph = speed_mps * 3.6

        accel_mps2 = 0.0
        if self._last_speed is not None and self._last_stamp is not None:
            dt = stamp - self._last_stamp
            if dt > 1e-4:
                candidate = (speed_mps - self._last_speed) / dt
                if (
                    self._max_abs_accel <= 0.0
                    or abs(candidate) <= self._max_abs_accel
                ):
                    accel_mps2 = candidate
                else:
                    accel_mps2 = self._last_accel

        self._last_speed = speed_mps
        self._last_stamp = stamp
        self._last_accel = accel_mps2
        voltage_drop = (
            self._nominal_voltage - voltage
            if math.isfinite(voltage)
            else math.nan
        )

        state = Float64MultiArray()
        state.data = [
            stamp,
            speed_mps,
            speed_kph,
            accel_mps2,
            erpm,
            motor_rpm,
            wheel_rpm,
            voltage,
            voltage_drop,
        ]
        self._publisher.publish(state)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VehicleMeasurementNode()
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
