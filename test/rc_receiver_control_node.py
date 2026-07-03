#!/usr/bin/env python3

import re
import time
from glob import glob
from typing import Dict, List

import rclpy
import serial
from geometry_msgs.msg import Twist
from rclpy.node import Node


class RcReceiverControlNode(Node):
    """Read Arduino Nano RC receiver data over USB and publish /cmd_vel.

    Expected Arduino serial line examples:
      1500,1498,1000,2000
      CH1:1500 CH2:1498 CH3:1000 CH4:2000
      ch1=1500,ch2=1498,ch3=1000,ch4=2000

    Channels are 1-based in ROS parameters because receiver labels are CH1, CH2...
    """

    def __init__(self):
        super().__init__("rc_receiver_control_node")

        self.port = self.declare_parameter("port", "auto").value
        self.baud = int(self.declare_parameter("baud", 115200).value)
        self.cmd_topic = self.declare_parameter("cmd_topic", "/cmd_vel").value

        self.probe_only = bool(self.declare_parameter("probe_only", True).value)
        self.channel_count = int(self.declare_parameter("channel_count", 8).value)

        self.steer_channel = int(self.declare_parameter("steer_channel", 1).value)
        self.throttle_channel = int(self.declare_parameter("throttle_channel", 3).value)
        # 0 disables arming. If used, keep the transmitter switch LOW until ready.
        self.arm_channel = int(self.declare_parameter("arm_channel", 0).value)
        self.arm_threshold_us = int(
            self.declare_parameter("arm_threshold_us", 1700).value
        )

        self.pwm_min_us = int(self.declare_parameter("pwm_min_us", 1000).value)
        self.pwm_center_us = int(self.declare_parameter("pwm_center_us", 1500).value)
        self.pwm_max_us = int(self.declare_parameter("pwm_max_us", 2000).value)
        self.deadband_us = int(self.declare_parameter("deadband_us", 40).value)
        self.throttle_zero_us = int(
            self.declare_parameter("throttle_zero_us", 1000).value
        )
        self.throttle_full_us = int(
            self.declare_parameter("throttle_full_us", 2000).value
        )
        self.throttle_deadband_us = int(
            self.declare_parameter("throttle_deadband_us", 30).value
        )
        self.throttle_bidirectional = bool(
            self.declare_parameter("throttle_bidirectional", True).value
        )

        self.invert_steer = bool(self.declare_parameter("invert_steer", False).value)
        self.invert_throttle = bool(
            self.declare_parameter("invert_throttle", True).value
        )

        # Keep RC VESC output slow. control_node.py multiplies this by
        # max_duty=0.20, so +/-0.25 becomes about +/-5% VESC duty at full stick.
        self.linear_cmd_max = float(self.declare_parameter("linear_cmd_max", 0.25).value)
        self.angular_cmd_max = float(self.declare_parameter("angular_cmd_max", 2.0).value)
        self.expo = float(self.declare_parameter("expo", 0.0).value)

        self.serial_timeout_sec = float(
            self.declare_parameter("serial_timeout_sec", 0.02).value
        )
        self.auto_probe_sec = float(self.declare_parameter("auto_probe_sec", 2.0).value)
        self.cmd_timeout_sec = float(self.declare_parameter("cmd_timeout_sec", 0.25).value)
        self.publish_hz = float(self.declare_parameter("publish_hz", 50.0).value)
        self.log_raw_sec = float(self.declare_parameter("log_raw_sec", 0.0).value)

        self.publisher = self.create_publisher(Twist, self.cmd_topic, 10)
        self.last_rx_time = 0.0
        self.last_log_time = 0.0
        self.latest_channels: Dict[int, int] = {}
        self.serial_buffer = bytearray()
        self.keyed_channel_re = re.compile(
            r"(?:ch)?\s*(\d+)\s*[:=]\s*(\d{3,4})", re.I
        )
        self.value_re = re.compile(r"\d{3,4}")

        self.port = self.resolve_serial_port(str(self.port))
        self.get_logger().info(f"Opening Arduino RC serial: {self.port} @ {self.baud}")
        self.serial = serial.Serial(
            self.port,
            self.baud,
            timeout=0.0,
        )
        time.sleep(2.0)
        self.serial.reset_input_buffer()

        timer_period = 1.0 / max(self.publish_hz, 1.0)
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            "RC receiver node started "
            f"(probe_only={self.probe_only}, steer=CH{self.steer_channel}, "
            f"throttle=CH{self.throttle_channel}, arm=CH{self.arm_channel})"
        )

    def timer_callback(self):
        self.read_available_lines()

        now = time.time()
        fresh = (now - self.last_rx_time) <= self.cmd_timeout_sec
        armed = self.is_armed()

        if self.should_log_raw(now):
            raw = self.format_channels(self.latest_channels)
            state = "PROBE" if self.probe_only else ("ARMED" if armed else "SAFE")
            self.get_logger().info(f"{state} raw: {raw}")

        if self.probe_only or not fresh or not armed:
            self.publish_cmd(0.0, 0.0)
            return

        steer_pwm = self.latest_channels.get(self.steer_channel)
        throttle_pwm = self.latest_channels.get(self.throttle_channel)
        if steer_pwm is None or throttle_pwm is None:
            self.publish_cmd(0.0, 0.0)
            return

        steer = self.pwm_to_unit(steer_pwm)
        throttle = self.pwm_to_unit(throttle_pwm)
        if not self.throttle_bidirectional:
            throttle = self.throttle_pwm_to_forward_unit(throttle_pwm)

        if self.invert_steer:
            steer *= -1.0
        if self.invert_throttle:
            throttle *= -1.0

        steer = self.apply_expo(steer)
        throttle = self.apply_expo(throttle)

        self.publish_cmd(
            throttle * self.linear_cmd_max,
            steer * self.angular_cmd_max,
        )

    def read_available_lines(self):
        waiting = self.serial.in_waiting
        if waiting <= 0:
            return

        self.serial_buffer.extend(self.serial.read(waiting))
        if len(self.serial_buffer) > 512:
            self.serial_buffer = self.serial_buffer[-512:]

        newline = self.serial_buffer.rfind(b"\n")
        if newline < 0:
            return

        data = self.serial_buffer[:newline]
        del self.serial_buffer[: newline + 1]

        lines = data.splitlines()
        if not lines:
            return

        line = lines[-1].decode(errors="ignore").strip()
        channels = self.parse_channels(line)
        if channels:
            self.latest_channels = channels
            self.last_rx_time = time.time()

    def resolve_serial_port(self, requested_port: str) -> str:
        if requested_port and requested_port.lower() != "auto":
            return requested_port

        by_id_ports = sorted(glob("/dev/serial/by-id/*"))
        preferred_keywords = (
            "arduino",
            "nano",
            "ch340",
            "ch341",
            "usb-serial",
            "usb_serial",
        )
        avoid_keywords = ("vesc",)

        for port in by_id_ports:
            low = port.lower()
            if any(word in low for word in avoid_keywords):
                continue
            if any(word in low for word in preferred_keywords):
                self.get_logger().info(f"Auto-selected Arduino USB port: {port}")
                return port

        candidates = sorted(glob("/dev/ttyUSB*")) + sorted(glob("/dev/ttyACM*"))
        probe_candidates = self.unique_ports(by_id_ports + candidates)
        detected = self.find_port_with_channel_output(probe_candidates)
        if detected:
            return detected

        if by_id_ports:
            known = ", ".join(by_id_ports)
            raise RuntimeError(
                "USB serial devices exist, but no port produced CH1/CH2/CH3 data. "
                "Pass the Arduino port explicitly with -p port:=... if you know it. "
                f"Found: {known}"
            )

        raise RuntimeError(
            "No Arduino USB serial port found. Plug Nano into Jetson USB or pass "
            "-p port:=/dev/ttyUSB0 (or the correct /dev/ttyACM*)"
        )

    def unique_ports(self, ports: List[str]) -> List[str]:
        unique: List[str] = []
        seen_targets = set()
        for port in ports:
            try:
                target = __import__("os").path.realpath(port)
            except OSError:
                target = port
            if target in seen_targets:
                continue
            seen_targets.add(target)
            unique.append(port)
        return unique

    def find_port_with_channel_output(self, ports: List[str]) -> str:
        channel_pattern = re.compile(r"\bCH\s*[1-9]\s*[:=]", re.I)

        for port in ports:
            try:
                ser = serial.Serial(
                    port,
                    self.baud,
                    timeout=min(max(self.serial_timeout_sec, 0.05), 0.2),
                )
            except Exception as exc:
                self.get_logger().warning(f"Auto-probe skip {port}: {exc}")
                continue

            deadline = time.time() + max(self.auto_probe_sec, 0.2)
            matched = False
            logged_lines = 0
            try:
                time.sleep(0.4)
                while time.time() < deadline:
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    if logged_lines < 3:
                        self.get_logger().info(f"Auto-probe {port}: {line[:120]}")
                        logged_lines += 1
                    if channel_pattern.search(line):
                        matched = True
                        break
            finally:
                ser.close()

            if matched:
                self.get_logger().info(f"Auto-selected RC receiver port: {port}")
                return port

            if logged_lines == 0:
                self.get_logger().info(f"Auto-probe {port}: no serial text")
            else:
                self.get_logger().warning(
                    f"Auto-probe {port}: serial text found, but not CHx receiver data"
                )

        return ""

    def parse_channels(self, line: str) -> Dict[int, int]:
        if not line:
            return {}

        keyed = self.keyed_channel_re.findall(line)
        if keyed:
            return {
                int(ch): int(value)
                for ch, value in keyed
                if self.pwm_min_us - 300 <= int(value) <= self.pwm_max_us + 300
            }

        values = [int(v) for v in self.value_re.findall(line)]
        if not values:
            return {}

        return {
            idx + 1: value
            for idx, value in enumerate(values[: self.channel_count])
            if self.pwm_min_us - 300 <= value <= self.pwm_max_us + 300
        }

    def pwm_to_unit(self, pwm_us: int) -> float:
        delta = pwm_us - self.pwm_center_us
        if abs(delta) <= self.deadband_us:
            return 0.0

        if delta > 0:
            span = max(self.pwm_max_us - self.pwm_center_us, 1)
        else:
            span = max(self.pwm_center_us - self.pwm_min_us, 1)

        return self.clamp(delta / span, -1.0, 1.0)

    def throttle_pwm_to_forward_unit(self, pwm_us: int) -> float:
        delta = pwm_us - self.throttle_zero_us
        if delta <= self.throttle_deadband_us:
            return 0.0

        span = max(self.throttle_full_us - self.throttle_zero_us, 1)
        return self.clamp(delta / span, 0.0, 1.0)

    def apply_expo(self, value: float) -> float:
        expo = self.clamp(self.expo, 0.0, 1.0)
        return (1.0 - expo) * value + expo * (value ** 3)

    def is_armed(self) -> bool:
        if self.arm_channel <= 0:
            return True
        arm_pwm = self.latest_channels.get(self.arm_channel)
        return arm_pwm is not None and arm_pwm >= self.arm_threshold_us

    def should_log_raw(self, now: float) -> bool:
        if self.log_raw_sec <= 0.0:
            return False
        if now - self.last_log_time < self.log_raw_sec:
            return False
        self.last_log_time = now
        return True

    def publish_cmd(self, speed: float, steering: float):
        msg = Twist()
        msg.linear.x = float(speed)
        msg.angular.z = float(steering)
        self.publisher.publish(msg)

    def format_channels(self, channels: Dict[int, int]) -> str:
        if not channels:
            return "(no valid channels yet)"
        ordered: List[str] = [
            f"CH{ch}={channels[ch]}" for ch in sorted(channels.keys())
        ]
        return " ".join(ordered)

    @staticmethod
    def clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)

    def destroy_node(self):
        self.get_logger().info("Stopping RC receiver node...")
        try:
            self.publish_cmd(0.0, 0.0)
            self.serial.close()
        except Exception as exc:
            self.get_logger().warning(f"Shutdown cleanup failed: {exc}")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = RcReceiverControlNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"rc_receiver_control_node failed: {exc}")
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
