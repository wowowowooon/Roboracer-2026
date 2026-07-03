#!/usr/bin/env python3
"""실차 주행 모니터 — python3 로 직접 실행 가능."""
from __future__ import annotations

import sys
from pathlib import Path


def _bootstrap_path() -> None:
    here = Path(__file__).resolve()
    pkg_root = here.parents[1]
    ws = here.parents[2]
    install_lib = ws / "install" / "path_following" / "lib"
    candidates = [pkg_root]
    if install_lib.is_dir():
        for pydir in sorted(install_lib.glob("python3.*")):
            candidates.insert(0, pydir / "site-packages")
    for p in candidates:
        if not p.is_dir():
            continue
        ps = str(p)
        if ps not in sys.path:
            sys.path.insert(0, ps)
        try:
            import path_following  # noqa: F401
            return
        except ImportError:
            continue
    print(
        "path_following 패키지를 찾지 못했습니다.\n"
        "  source /home/nvidia/f1tenth_ajou/install/setup.bash\n"
        "  또는: ros2 run path_following drive_monitor",
        file=sys.stderr,
    )
    sys.exit(1)


_bootstrap_path()
from path_following.drive_monitor import main

if __name__ == "__main__":
    main()
