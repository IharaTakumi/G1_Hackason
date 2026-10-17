"""地図を作ったときに歩いた軌跡に沿って、見回りの目的地を選ぶ。ROS 不要。

軌跡を --spacing m ごとに区切り、各点のまわり --search m の範囲で「通れる場所のうち、障害物から最も離れたセル」に置き直す。
Nav2 は障害物の近くの目的地に近づけないことがある（体の半径 約 0.32 m）ので、余裕のある点を選ぶため。

    python3 pick_waypoints.py --map navmap.yaml --trajectory trajectory.tum
    python3 pick_waypoints.py --map navmap.yaml --trajectory trajectory.tum --reverse --out patrol.txt

出力: 1 行に `x y  # 障害物まで d m`。最後に send_goals.py にそのまま渡せる並び（x1 y1 x2 y2 ...）を出す。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from maplib import load_grid, load_trajectory  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, required=True)
    ap.add_argument("--trajectory", type=Path, required=True, help="地図を作ったときの軌跡（TUM 形式）")
    ap.add_argument("--spacing", type=float, default=2.6, help="目的地の間隔（軌跡に沿った距離 [m]）")
    ap.add_argument("--search", type=float, default=0.5, help="置き直す範囲の半径 [m]")
    ap.add_argument("--skip-start", type=float, default=2.5, help="軌跡の始めのこの距離 [m] は使わない（出発点のそば）")
    ap.add_argument("--skip-end", type=float, default=1.5, help="軌跡の終わりのこの距離 [m] は使わない")
    ap.add_argument("--warn-clearance", type=float, default=0.5, help="障害物までこれ未満なら警告する [m]")
    ap.add_argument("--reverse", action="store_true", help="逆回りの順にする")
    ap.add_argument("--out", type=Path, help="目的地を書き出すファイル（render_map / live_map の --points-file に使える）")
    args = ap.parse_args()

    grid = load_grid(args.map)
    traj = load_trajectory(args.trajectory)
    if traj is None:
        print("[pick_waypoints] 軌跡が空")
        return 1
    xy = traj[:, 1:3]
    dist = np.r_[0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]
    rad = int(round(args.search / grid.resolution))
    pts: list[tuple[float, float, float]] = []
    for s in np.arange(args.skip_start, dist[-1] - args.skip_end, args.spacing):
        x, y = xy[min(np.searchsorted(dist, s), len(xy) - 1)]
        rc = grid.cell(x, y)
        if rc is None:
            continue
        best: tuple[float, int, int] | None = None
        for dr in range(-rad, rad + 1):
            for dc in range(-rad, rad + 1):
                r, c = rc[0] + dr, rc[1] + dc
                if dr * dr + dc * dc > rad * rad or not (0 <= r < grid.cls.shape[0] and 0 <= c < grid.cls.shape[1]):
                    continue
                if grid.cls[r, c] == 0 and (best is None or grid.clearance[r, c] > best[0]):
                    best = (float(grid.clearance[r, c]), r, c)
        if best is not None:
            bx, by = grid.xy(best[1], best[2])
            pts.append((round(bx, 1), round(by, 1), best[0]))
    if args.reverse:
        pts.reverse()
    print(f"[pick_waypoints] 軌跡 {dist[-1]:.1f} m から {len(pts)} か所を選んだ（{'逆回り' if args.reverse else '順回り'}）")
    lines = []
    for i, (x, y, d) in enumerate(pts, 1):
        warn = "  ⚠ 狭い" if d < args.warn_clearance else ""
        print(f"  {i:2d}: ({x:5.1f}, {y:5.1f})  障害物まで {d:.2f} m{warn}")
        lines.append(f"{x:.1f} {y:.1f}  # 障害物まで {d:.2f} m")
    if args.out:
        args.out.write_text("\n".join(lines) + "\n")
        print(f"[pick_waypoints] → {args.out}")
    print("[pick_waypoints] send_goals.py に渡す並び:")
    print(" ".join(f"{x:.1f} {y:.1f}" for x, y, _ in pts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
