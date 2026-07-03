"""장애물 게이트 — local_planner 전용 (static 출력 필터)."""
from __future__ import annotations

import math
from typing import List, Tuple

from path_following.track_sliding import lateral_distance_to_closed_polyline


def filter_obstacles_laser_frame(
    obstacle_data: list,
    *,
    forward_min_m: float,
    forward_max_m: float,
    lateral_abs_max_m: float,
    corridor_enable: bool,
    corridor_max_lat_m: float,
    track_pts: List[Tuple[float, float]],
    laser_to_map,
) -> list:
    """
    /static_obstacles [id,x,y,r,...] (laser) → planner 게이트 통과분만.
    laser_to_map: (lx, ly) -> (mx, my) or None if TF unavailable.
    """
    if len(obstacle_data) < 4:
        return []

    out: list = []
    n = len(obstacle_data) // 4
    for i in range(n):
        base = 4 * i
        oid = obstacle_data[base]
        x = float(obstacle_data[base + 1])
        y = float(obstacle_data[base + 2])
        r = float(obstacle_data[base + 3])

        if x < forward_min_m or x > forward_max_m:
            continue
        if abs(y) > lateral_abs_max_m:
            continue

        if corridor_enable and track_pts and laser_to_map is not None:
            mapped = laser_to_map(x, y)
            if mapped is None:
                continue
            mx, my = mapped
            if lateral_distance_to_closed_polyline(mx, my, track_pts) > corridor_max_lat_m:
                continue

        out.extend([float(oid), x, y, r])

    return out


def closest_obstacle_surface_m(
    obstacle_data: list,
    *,
    forward_cone_rad: float | None = None,
    min_forward_x_m: float = 0.0,
    lateral_abs_max_m: float | None = None,
) -> float:
    """필터된 장애 목록에서 전방 콘·거리 기준 최근접 표면 거리(m)."""
    if len(obstacle_data) < 4:
        return float("inf")
    n = len(obstacle_data) // 4
    best = float("inf")
    for i in range(n):
        x = float(obstacle_data[4 * i + 1])
        y = float(obstacle_data[4 * i + 2])
        r = float(obstacle_data[4 * i + 3])
        if x < min_forward_x_m:
            continue
        if lateral_abs_max_m is not None and abs(y) > lateral_abs_max_m:
            continue
        if forward_cone_rad is not None:
            if x <= 0.0:
                continue
            angle = math.atan2(y, x)
            if abs(angle) > forward_cone_rad:
                continue
        d = math.hypot(x, y) - r
        if d < best:
            best = d
    return max(0.0, best) if best != float("inf") else float("inf")


def obstacles_remain_for_avoid(
    obstacle_data: list,
    *,
    pass_rear_x_m: float,
    lateral_abs_max_m: float,
) -> bool:
    """
    True if any gate obstacle is not fully behind the vehicle (laser frame).
    Rear edge x-r must be <= pass_rear_x_m (e.g. -0.35) to count as cleared.
    """
    if len(obstacle_data) < 4:
        return False
    n = len(obstacle_data) // 4
    for i in range(n):
        x = float(obstacle_data[4 * i + 1])
        y = float(obstacle_data[4 * i + 2])
        r = float(obstacle_data[4 * i + 3])
        if abs(y) > lateral_abs_max_m:
            continue
        if (x - r) > pass_rear_x_m:
            return True
    return False
