#!/usr/bin/env python3
"""Publish map->base_link until Cartographer has an active localization trajectory."""

from __future__ import annotations

import math

import rclpy
from cartographer_ros_msgs.srv import GetTrajectoryStates
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

_TRAJECTORY_ACTIVE = 0


def _yaw_from_pose(msg: PoseStamped) -> float:
    q = msg.pose.orientation
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


class LocalizationTfBootstrap(Node):
    def __init__(self):
        super().__init__('localization_tf_bootstrap')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_hz', 30.0)

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        hz = max(1.0, float(self.get_parameter('publish_hz').value))

        self._broadcaster = TransformBroadcaster(self)
        self._states_client = self.create_client(
            GetTrajectoryStates, '/get_trajectory_states'
        )
        self._states_future = None
        self._active_trajectory = False
        self._had_active_trajectory = False
        self._last_x = 0.0
        self._last_y = 0.0
        self._last_yaw = 0.0

        self.create_subscription(PoseStamped, '/pose', self._on_pose, 10)
        self.create_timer(1.0 / hz, self._on_timer)
        self.create_timer(0.35, self._poll_trajectory_states)
        self.get_logger().info(
            f'TF bootstrap: {self._map_frame}->{self._base_frame} until Localization OK'
        )

    def _on_pose(self, msg: PoseStamped) -> None:
        frame = msg.header.frame_id
        if frame and frame not in (self._map_frame, ''):
            return
        self._last_x = msg.pose.position.x
        self._last_y = msg.pose.position.y
        self._last_yaw = _yaw_from_pose(msg)

    def _poll_trajectory_states(self) -> None:
        if self._states_future is not None:
            return
        if not self._states_client.service_is_ready():
            return
        self._states_future = self._states_client.call_async(
            GetTrajectoryStates.Request()
        )
        self._states_future.add_done_callback(self._on_trajectory_states)

    def _on_trajectory_states(self, future) -> None:
        self._states_future = None
        active = False
        try:
            resp = future.result()
            if resp is not None and resp.status.code == 0:
                for state in resp.trajectory_states.trajectory_state:
                    if state == _TRAJECTORY_ACTIVE:
                        active = True
                        break
        except Exception as exc:
            self.get_logger().debug(f'get_trajectory_states: {exc}')
        self._active_trajectory = active
        if active:
            self._had_active_trajectory = True

    def _publish(self, x: float, y: float, yaw: float) -> None:
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self._map_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.z = math.sin(yaw * 0.5)
        t.transform.rotation.w = math.cos(yaw * 0.5)
        self._broadcaster.sendTransform(t)

    def _on_timer(self) -> None:
        if self._active_trajectory:
            return
        if self._had_active_trajectory:
            self._publish(self._last_x, self._last_y, self._last_yaw)
        else:
            self._publish(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationTfBootstrap()
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
