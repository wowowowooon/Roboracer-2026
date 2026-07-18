#!/usr/bin/env python3
"""
control_node 테스트용 /drive 발행기.

localization / path_following 없이:
  - 이 노드: /drive 에 AckermannDriveStamped 발행 (직진 기본)
  - control_node: CH5 자율일 때만 /drive 조향 사용, 속도는 control_node CFG 가 통제
  - CH5 수동: control_node 가 /drive 무시 → RC만 동작

사용 (젯슨):
  ros2 run path_following control_node
  ros2 run path_following straight_drive_publisher

튜닝은 아래 CFG 또는:
  ros2 param set /straight_drive_publisher steering_angle_rad 0.05
  ros2 param set /straight_drive_publisher enabled false   # 발행 중지
"""
from __future__ import annotations

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node


# ============================================================
# USER TUNING — 여기만 수정
# ============================================================
CFG = {
    "drive_topic": "/drive",
    "publish_hz": 40.0,           # control_node cmd_timeout(0.25s) 보다 충분히 빠름
    "enabled": True,
    # control_node AUTO 는 /drive.speed 를 무시하고 max_target_speed_mps 사용.
    # 여기 speed 는 메시지/로그용 (나중에 control 이 speed 를 쓰면 그대로 활용).
    "speed_mps": 1.0,
    "steering_angle_rad": 0.0,    # 0 = 직진. + = 좌(Stanley 관례)
    "frame_id": "base_link",
}


def _param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


class StraightDrivePublisher(Node):
    def __init__(self) -> None:
        super().__init__("straight_drive_publisher")

        for key, value in CFG.items():
            self.declare_parameter(key, value)

        self._drive_topic = str(self.get_parameter("drive_topic").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        hz = max(1.0, float(self.get_parameter("publish_hz").value))

        self._pub = self.create_publisher(AckermannDriveStamped, self._drive_topic, 10)
        self.create_timer(1.0 / hz, self._tick)

        self.get_logger().info(
            f"straight_drive_publisher → `{self._drive_topic}` @ {hz:.0f}Hz | "
            f"steer={float(self.get_parameter('steering_angle_rad').value):+.3f}rad "
            f"speed_field={float(self.get_parameter('speed_mps').value):.2f}m/s "
            f"(AUTO에서 실제 속도는 control_node max_target_speed_mps)"
        )
        self.get_logger().info(
            "MANUAL(CH5): control_node 가 /drive 무시. AUTO(CH5): 이 조향 명령 사용."
        )

    def _tick(self) -> None:
        if not _param_bool(self.get_parameter("enabled").value):
            return

        speed = float(self.get_parameter("speed_mps").value)
        steer = float(self.get_parameter("steering_angle_rad").value)
        if not math.isfinite(speed):
            speed = 0.0
        if not math.isfinite(steer):
            steer = 0.0

        msg = AckermannDriveStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        msg.drive.speed = speed
        msg.drive.steering_angle = steer
        self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StraightDrivePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
