#!/usr/bin/env python3
"""README 그림을 레포 안의 데이터와 코드로 다시 그린다.

    python3 tools/make_figures.py

- track_lines_*.png      : Cartographer 맵 위 센터라인 / 최소곡률 레이스라인 (속도 색)
- speed_profile_*.png    : 레이스라인 진행거리 s 에 따른 목표 속도
- offset_maneuver_*.png  : 같은 장애물을 속도별로 피하는 횡오프셋 기동 d(s)

각각 _light / _dark 두 벌, 투명 배경.

외부 데이터 없이 maps/, src/path_following/config/, offset_maneuver.py 만 쓴다.
"""
from __future__ import annotations

import csv
import math
import os
import sys

import matplotlib
import matplotlib.patches

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402
from matplotlib.colors import LinearSegmentedColormap  # noqa: E402
from matplotlib.collections import LineCollection  # noqa: E402
from PIL import Image  # noqa: E402

ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
PKG = os.path.join(ROOT, "src", "path_following")
OUT = os.path.join(ROOT, "docs", "images")
MAP_YAML = os.path.join(ROOT, "maps", "cartographer_map_20260822_193736_rosmap.yaml")

sys.path.insert(0, PKG)
from path_following import offset_maneuver as om  # noqa: E402
from path_following import vehicle_geometry as vg  # noqa: E402

# GitHub README 는 라이트/다크 테마가 있어서 두 벌을 투명 배경으로 뽑고
# <picture> 로 고른다. 값은 dataviz reference palette 의 light/dark 스텝.
THEMES = {
    "light": dict(
        SURFACE="#ffffff",
        INK="#1f2328",
        INK_2="#59636e",
        GRID="#d1d9e0",
        WALL=(0x1F, 0x23, 0x28),
        SERIES=["#2a78d6", "#eb6834", "#1baf7a"],
        SEQ=["#9ec5f4", "#5598e7", "#2a78d6", "#1c5cab", "#0d366b"],
    ),
    "dark": dict(
        SURFACE="#0d1117",
        INK="#f0f6fc",
        INK_2="#9198a1",
        GRID="#3d444d",
        WALL=(0xF0, 0xF6, 0xFC),
        SERIES=["#3987e5", "#d95926", "#199e70"],
        SEQ=["#184f95", "#2a78d6", "#5598e7", "#9ec5f4", "#cde2fb"],
    ),
}
THEME = "light"


def apply_theme(name):
    global THEME, SURFACE, INK, INK_2, GRID, WALL, SERIES, SEQ
    THEME = name
    th = THEMES[name]
    SURFACE, INK, INK_2, GRID, WALL, SERIES = (
        th["SURFACE"], th["INK"], th["INK_2"], th["GRID"], th["WALL"], th["SERIES"]
    )
    SEQ = LinearSegmentedColormap.from_list("seq", th["SEQ"])
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "text.color": INK,
            "axes.edgecolor": GRID,
            "axes.labelcolor": INK_2,
            "xtick.color": INK_2,
            "ytick.color": INK_2,
            "axes.facecolor": "none",
            "figure.facecolor": "none",
            "savefig.facecolor": "none",
            "axes.grid": True,
            "grid.color": GRID,
            "grid.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.facecolor": SURFACE,
            "legend.edgecolor": GRID,
            "legend.framealpha": 0.9,
        }
    )


def save(fig, stem):
    fig.savefig(os.path.join(OUT, f"{stem}_{THEME}.png"), dpi=160, transparent=True)
    plt.close(fig)


def map_overlay(img):
    """점유 격자 → 벽만 잉크색으로 남긴 RGBA. 빈 공간은 투명."""
    occ = 1.0 - img.astype(float) / 255.0          # 0 = free, 1 = wall
    rgba = np.zeros(img.shape + (4,))
    rgba[..., :3] = np.array(WALL) / 255.0
    rgba[..., 3] = np.clip(occ, 0.0, 1.0) * 0.55
    return rgba


def load_xyv(path):
    with open(path) as f:
        rows = list(csv.DictReader(f))
    xy = np.array([[float(r["x"]), float(r["y"])] for r in rows])
    v = np.array([float(r["v"]) for r in rows]) if "v" in rows[0] else None
    return xy, v


def arc_length(xy):
    seg = np.linalg.norm(np.diff(np.vstack([xy, xy[:1]]), axis=0), axis=1)
    return np.concatenate([[0.0], np.cumsum(seg)[:-1]]), seg.sum()


def fig_track():
    with open(MAP_YAML) as f:
        meta = yaml.safe_load(f)
    img = np.asarray(Image.open(os.path.join(os.path.dirname(MAP_YAML), meta["image"])).convert("L"))
    res = meta["resolution"]
    ox, oy = meta["origin"][:2]
    h, w = img.shape
    extent = [ox, ox + w * res, oy, oy + h * res]

    center, _ = load_xyv(os.path.join(PKG, "config", "centerline.csv"))
    race, v = load_xyv(os.path.join(PKG, "config", "raceline.csv"))

    pad = 0.8
    allxy = np.vstack([center, race])
    xmin, ymin = allxy.min(0) - pad
    xmax, ymax = allxy.max(0) + pad

    fig, ax = plt.subplots(figsize=(8.0, 8.0 * (ymax - ymin) / (xmax - xmin) + 0.6))
    ax.imshow(map_overlay(img), extent=extent, origin="upper", interpolation="bilinear")
    c = np.vstack([center, center[:1]])
    ax.plot(c[:, 0], c[:, 1], color=INK_2, lw=1.2, ls=(0, (4, 3)), label="Centerline (wall-to-wall)")

    r = np.vstack([race, race[:1]])
    vv = np.concatenate([v, v[:1]])
    segs = np.stack([r[:-1], r[1:]], axis=1)
    lc = LineCollection(segs, cmap=SEQ, linewidths=3.0, capstyle="round")
    lc.set_array(0.5 * (vv[:-1] + vv[1:]))
    lc.set_clim(v.min(), v.max())
    ax.add_collection(lc)
    ax.plot([], [], color=SERIES[0], lw=3, label="Min-curvature raceline")

    ax.annotate(
        "start / dir",
        xy=race[0],
        xytext=race[0] + (race[6] - race[0]) * 0 + np.array([0.5, 0.4]),
        color=INK_2,
        fontsize=9,
        arrowprops=dict(arrowstyle="-", color=INK_2, lw=0.8),
    )
    ax.annotate(
        "", xy=race[12], xytext=race[0], arrowprops=dict(arrowstyle="-|>", color=INK, lw=1.4)
    )

    ax.set_xlim(xmin, xmax)
    ax.set_ylim(ymin, ymax)
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]  (map frame)")
    ax.set_ylabel("y [m]")
    ax.grid(False)
    cb = fig.colorbar(lc, ax=ax, fraction=0.035, pad=0.02)
    cb.set_label("target speed [m/s]", color=INK_2)
    cb.outline.set_edgecolor(GRID)
    ax.legend(loc="upper left", fontsize=9)

    _, Lc = arc_length(center)
    _, Lr = arc_length(race)
    ax.set_title(
        f"Track map + reference lines   centerline {Lc:.2f} m  →  raceline {Lr:.2f} m",
        color=INK,
        fontsize=11,
        loc="left",
    )
    fig.tight_layout()
    save(fig, "track_lines")


def fig_speed():
    race, v = load_xyv(os.path.join(PKG, "config", "raceline.csv"))
    s, L = arc_length(race)
    fig, ax = plt.subplots(figsize=(8.0, 2.8))
    ax.fill_between(s, 0, v, color=SERIES[0], alpha=0.15, lw=0)
    ax.plot(s, v, color=SERIES[0], lw=2)
    i = int(np.argmin(v))
    ax.annotate(
        f"v_min floor {v[i]:.1f} m/s",
        xy=(s[i], v[i]),
        xytext=(s[i] + 1.5, v[i] - 1.2),
        color=INK_2,
        fontsize=9,
        arrowprops=dict(arrowstyle="-", color=INK_2, lw=0.8),
    )
    ax.set_xlim(0, L)
    ax.set_ylim(0, v.max() + 0.8)
    ax.set_xlabel("distance along raceline s [m]")
    ax.set_ylabel("v [m/s]")
    ax.set_title(
        "Offline speed profile   v = min(v_max, √(a_lat/κ)) + forward/backward accel passes",
        color=INK,
        fontsize=11,
        loc="left",
    )
    fig.tight_layout()
    save(fig, "speed_profile")


def fig_maneuver():
    cfg = om.ManeuverConfig(
        half_width_m=vg.HALF_WIDTH_M,
        lateral_margin_m=0.12,
        max_offset_m=0.65,
        a_lat_enter_mps2=3.0,
        a_lat_exit_mps2=2.0,
        a_lat_hard_mps2=4.5,
        enter_min_m=1.0,
        enter_max_m=9.0,
        exit_min_m=1.5,
        exit_max_m=12.0,
        hold_front_m=vg.FRONT_M + 0.20,
        hold_rear_m=vg.LENGTH_M + 0.30,
        merge_gap_m=3.0,
        v_plan_min_mps=1.5,
        max_steer_rad=0.3735,
        wheelbase_m=vg.WHEELBASE_M,
    )
    obs = om.ObstacleSD(s=12.0, d=0.0, r=0.20)

    fig, ax = plt.subplots(figsize=(8.0, 3.4))
    ax.add_patch(
        matplotlib.patches.Rectangle(
            (obs.s - obs.r, obs.d - obs.r), 2 * obs.r, 2 * obs.r, color=INK_2, alpha=0.45, lw=0
        )
    )
    ax.text(obs.s, -obs.r - 0.05, "obstacle", ha="center", va="top", color=INK_2, fontsize=9)
    ax.axhline(0, color=INK_2, lw=0.8, ls=(0, (4, 3)))
    ax.text(0.2, 0.03, "raceline (d = 0)", color=INK_2, fontsize=9, va="bottom")

    for color, v in zip(SERIES, (3.0, 5.0, 7.0)):
        m = om.plan_maneuver([obs], cfg, d_ego=0.0, d_ego_prime=0.0, v=v)
        if m is None:
            continue
        ds = np.linspace(0, m.total_length_m + 1.0, 600)
        d = np.array([m.d_at(x) for x in ds])
        steer = math.degrees(
            om.steering_for_offset(om.D2_PEAK * abs(m.d_pass) / m.enter_len_m**2, vg.WHEELBASE_M)
        )
        label = (
            f"{v:.0f} m/s   entry {m.enter_len_m:3.1f} m   "
            f"a_lat {m.peak_lateral_accel_mps2:.1f} m/s²   steer {steer:.1f}°"
        )
        ax.plot(ds, d, color=color, lw=2, label=label)

    ax.set_xlim(0, 24)
    ax.set_ylim(-0.35, 1.0)
    ax.set_xlabel("distance ahead of ego  s [m]")
    ax.set_ylabel("lateral offset d [m]")
    ax.legend(
        loc="upper right",
        prop={"family": "DejaVu Sans Mono", "size": 8.5},
    )
    ax.set_title(
        "Offset maneuver: length scales with speed  (L = v·√(5.77·Δd / a_lat))",
        color=INK,
        fontsize=11,
        loc="left",
    )
    fig.tight_layout()
    save(fig, "offset_maneuver")


def main():
    os.makedirs(OUT, exist_ok=True)
    for name in THEMES:
        apply_theme(name)
        fig_track()
        fig_speed()
        fig_maneuver()
    print("wrote", sorted(os.listdir(OUT)))


if __name__ == "__main__":
    main()
