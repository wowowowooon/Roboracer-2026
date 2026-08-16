#!/usr/bin/env python3
"""한 바퀴 주행 기록기 — /stanley/debug 를 CSV 로 받아 적는다.

가속 구간 라인 홀드가 실제로 켜지는지, 켜졌을 때 cte 가 줄어드는지 보려고
만든 일회용 스크립트다. 노드 코드는 건드리지 않는다.
"""
from __future__ import annotations

import csv
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

COLS = [
    "t_sec",
    "cte_m",
    "hdg_err_rad",
    "hdg_term_rad",
    "cte_term_rad",
    "fb_sum_rad",
    "steer_raw_rad",
    "steer_cmd_rad",
    "speed_mps",
    "path_idx",
    "kappa_1pm",
    "ff_rad",
    "before_sat_rad",
    "lat_active_m",
    "path_x",
    "path_y",
    "csv_lat_m",
    "veh_x",
    "veh_y",
    "veh_yaw_rad",
    "accel_x_mps2",
    "accel_u",
    "accel_add_rad",
]


class LapRecorder(Node):
    def __init__(self, out_path: str):
        super().__init__("tmp_lap_recorder")
        self._t0 = time.time()
        self._n = 0
        self._short = 0
        self._f = open(out_path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(COLS)
        self.create_subscription(
            Float64MultiArray, "/stanley/debug", self._cb, 50
        )
        self.create_timer(2.0, self._tick)
        self.get_logger().info(f"기록 시작 → {out_path}")

    def _cb(self, msg: Float64MultiArray) -> None:
        d = list(msg.data)
        if len(d) < 19:
            return
        if len(d) < 23 - 1:  # 22 값(가속 3개 포함) 미만이면 구버전 바이너리
            self._short += 1
            d = d + [float("nan")] * (22 - len(d))
        row = [round(time.time() - self._t0, 3)] + [
            (round(v, 6) if math.isfinite(v) else "") for v in d[:22]
        ]
        self._w.writerow(row)
        self._n += 1

    def _tick(self) -> None:
        self._f.flush()
        if self._n == 0:
            self.get_logger().warn("아직 /stanley/debug 수신 없음")
            return
        tag = " [구버전 바이너리! 재기동 필요]" if self._short else ""
        self.get_logger().info(f"{self._n} 샘플 기록{tag}")

    def close(self) -> None:
        self._f.flush()
        self._f.close()


def main() -> None:
    out = sys.argv[1] if len(sys.argv) > 1 else "lap_debug.csv"
    out = os.path.abspath(out)
    rclpy.init()
    node = LapRecorder(out)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.get_logger().info(f"기록 종료: {out}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
