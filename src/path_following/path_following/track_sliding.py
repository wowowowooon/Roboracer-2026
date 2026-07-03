"""폐곡선 CSV 상의 투영 + 슬라이딩 윈도우 (local_planner · stanley 공용)."""
from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import List, Tuple

_DEFAULT_CSV_NAMES = ("raceline.csv", "centerline.csv")


def resolve_csv_path(csv_param: str) -> str:
    """CFG csv_path 가 비어 있으면 config/raceline.csv → centerline.csv 자동 탐색."""
    p = (csv_param or "").strip()
    if p:
        return p

    roots: list[Path] = []
    try:
        from ament_index_python.packages import get_package_share_directory

        roots.append(Path(get_package_share_directory("path_following")) / "config")
    except Exception:
        pass

    here = Path(__file__).resolve()
    for parent in here.parents:
        if parent.name == "path_following" and (parent / "package.xml").is_file():
            roots.append(parent / "config")
            break

    seen: set[Path] = set()
    for root in roots:
        root = root.resolve()
        if root in seen:
            continue
        seen.add(root)
        for name in _DEFAULT_CSV_NAMES:
            cand = root / name
            if cand.is_file():
                return str(cand)

    names = ", ".join(_DEFAULT_CSV_NAMES)
    raise FileNotFoundError(
        f"path_following/config/ 에서 {names} 을 찾지 못했습니다. "
        "scripts 로 생성하거나 config/ 에 CSV 를 넣은 뒤 "
        "`colcon build --packages-select path_following` 하세요."
    )


def param_bool(val) -> bool:
    if isinstance(val, bool):
        return val
    return str(val).lower() in ("1", "true", "yes")


def load_csv_xy(path: str) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    with open(path, "r", newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        rows = list(reader)
    start = 0
    for i, r in enumerate(rows):
        if not r or r[0].strip().startswith("#"):
            start = i + 1
            continue
        if len(r) < 2:
            continue
        try:
            x = float(r[0].strip())
            y = float(r[1].strip())
            pts.append((x, y))
        except ValueError:
            if i == start and (
                "x" in (r[0] + r[1]).lower() or "m" in (r[0] + r[1]).lower()
            ):
                start = i + 1
            continue
    return pts


def apply_track_direction(
    points: List[Tuple[float, float]], reverse: bool
) -> List[Tuple[float, float]]:
    """폐곡선 CSV 진행 방향 반전 (로컬 pose yaw 와 경로 tangent 불일치 시)."""
    if not reverse or len(points) < 2:
        return points
    return list(reversed(points))


def _closest_point_on_segment(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> Tuple[float, float, float]:
    abx, aby = bx - ax, by - ay
    apx, apy = px - ax, py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-14:
        return ax, ay, 0.0
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    return ax + t * abx, ay + t * aby, t


def lateral_distance_to_closed_polyline(
    mx: float, my: float, pts: List[Tuple[float, float]]
) -> float:
    """
    맵 평면에서 점 (mx,my) 과 폐폴리라인(pts) 사이 최단 거리(m).
    트랙 코리도 필터: 레이스라인에 가깝지 않으면(벽 등) 큰 값.
    """
    n = len(pts)
    if n < 2:
        return float("inf")
    best_d2 = float("inf")
    for i in range(n):
        ax, ay = pts[i]
        bx, by = pts[(i + 1) % n]
        qx, qy, _t = _closest_point_on_segment(mx, my, ax, ay, bx, by)
        d2 = (mx - qx) ** 2 + (my - qy) ** 2
        if d2 < best_d2:
            best_d2 = d2
    return math.sqrt(best_d2)


class LoopTrackSliding:
    """맵 평면 폐선 궤적 + 앵커 검색 폭으로 슬라이딩 N점."""

    def __init__(
        self,
        points: List[Tuple[float, float]],
        path_window_size: int,
        path_anchor_half_width: int,
    ) -> None:
        if len(points) < 2:
            raise ValueError("LoopTrackSliding needs ≥2 points")
        self.points = points
        self.path_window_size = max(10, int(path_window_size))
        self.path_anchor_half_width = max(30, int(path_anchor_half_width))
        self._track_anchor_seg = 0
        self._anchor_initialized = False

    def reset_anchor(self) -> None:
        self._anchor_initialized = False
        self._track_anchor_seg = 0

    def closest_projection_on_loop(self, mx: float, my: float) -> Tuple[float, float, int]:
        pts = self.points
        n = len(pts)
        half = self.path_anchor_half_width

        def eval_seg(i: int) -> Tuple[float, float, float]:
            ax, ay = pts[i]
            bx, by = pts[(i + 1) % n]
            qx, qy, _t = _closest_point_on_segment(mx, my, ax, ay, bx, by)
            d2 = (mx - qx) ** 2 + (my - qy) ** 2
            return qx, qy, d2

        best_qx, best_qy = 0.0, 0.0
        best_seg = 0
        best_d2 = float("inf")

        if not self._anchor_initialized:
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i
            self._anchor_initialized = True
        else:
            for k in range(-half, half + 1):
                i = (self._track_anchor_seg + k) % n
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i

        if best_d2 > 100.0:
            best_d2 = float("inf")
            for i in range(n):
                qx, qy, d2 = eval_seg(i)
                if d2 < best_d2:
                    best_d2 = d2
                    best_qx, best_qy = qx, qy
                    best_seg = i

        self._track_anchor_seg = best_seg
        return (best_qx, best_qy, best_seg)

    def sliding_xy(self, mx: float, my: float) -> List[Tuple[float, float]]:
        n = len(self.points)
        px, py, seg_i = self.closest_projection_on_loop(mx, my)
        w = min(self.path_window_size, n)
        out: List[Tuple[float, float]] = [(px, py)]
        for k in range(1, w):
            out.append(self.points[(seg_i + k) % n])
        return out
