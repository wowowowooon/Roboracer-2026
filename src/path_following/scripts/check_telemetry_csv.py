#!/usr/bin/env python3
"""Summarize completeness and timing of the newest telemetry CSV."""

from __future__ import annotations

import argparse
import csv
import glob
import math
import os
import statistics


VALID_COLUMNS = (
    "data_valid",
    "vesc_valid",
    "vehicle_valid",
    "rc_valid",
    "safety_valid",
)


def newest_csv(root: str) -> str:
    pattern = os.path.join(os.path.expanduser(root), "*", "telemetry.csv")
    candidates = glob.glob(pattern)
    if not candidates:
        raise FileNotFoundError(f"No telemetry.csv found under {root}")
    return max(candidates, key=os.path.getmtime)


def is_true(value: str) -> bool:
    return value.strip().lower() in ("true", "1", "yes")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", nargs="?")
    parser.add_argument("--root", default="~/f1tenth_ajou/logs")
    args = parser.parse_args()
    requested_path = args.csv_path or newest_csv(args.root)
    path = os.path.abspath(os.path.expanduser(requested_path))

    with open(path, newline="", encoding="utf-8-sig") as csv_file:
        rows = list(csv.DictReader(csv_file))

    fieldnames = list(rows[0].keys()) if rows else []
    empty_columns = [
        column
        for column in fieldnames
        if any(row.get(column, "").strip() == "" for row in rows)
    ]
    times = []
    for row in rows:
        try:
            value = float(row["time_sec"])
            if math.isfinite(value):
                times.append(value)
        except (KeyError, TypeError, ValueError):
            pass
    dts = [b - a for a, b in zip(times, times[1:]) if b >= a]

    print(f"CSV: {path}")
    print(f"Total rows: {len(rows)}")
    print(
        "Columns containing empty values: "
        + (", ".join(empty_columns) if empty_columns else "NONE")
    )
    for column in VALID_COLUMNS:
        if column in fieldnames and rows:
            ratio = sum(is_true(row[column]) for row in rows) / len(rows)
            print(f"{column} ratio: {ratio:.1%}")
    print(
        f"Average logging dt: {statistics.fmean(dts):.6f} s"
        if dts
        else "Average logging dt: NaN"
    )
    faults = sorted({row.get("fault_code", "NONE") for row in rows})
    reasons = sorted({row.get("limit_reason", "NONE") for row in rows})
    print("fault_code values: " + (", ".join(faults) if faults else "NONE"))
    reason_values = ", ".join(reasons) if reasons else "NONE"
    print("limit_reason values: " + reason_values)


if __name__ == "__main__":
    main()
