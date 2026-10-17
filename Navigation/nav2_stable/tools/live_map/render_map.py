"""Nav2 用の地図（.yaml + .pgm）を、人が見やすい画像（PNG）にする。ROS 不要。

    python3 render_map.py --map navmap.yaml --out map_readable.png
    python3 render_map.py --map navmap.yaml --trajectory trajectory.tum --points 3.0 -0.6 10.0 -0.2 \\
        --pose -0.40 -0.39 -34 --out map_with_goals.png
    python3 render_map.py --map navmap.yaml --trajectory trajectory.tum --points-file patrol.txt --loop \\
        --xlim -2 15 --ylim -6 2 --title "見回りルート" --out patrol_route.png

描くもの: 通れる場所（白）・狭い所（薄いオレンジ。障害物まで --narrow m 未満）・障害物（濃い紺）・未知（灰）、
1 m ごとの目盛り、2 m の物差し、地図を作ったときの軌跡（青）、原点（緑、+x の矢印）、目的地（番号付き）、
現在位置（赤、--pose）、凡例。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.lines import Line2D  # noqa: E402
from matplotlib.patches import Patch  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from maplib import (COLOR_NARROW, COLOR_OCCUPIED, COLOR_UNKNOWN, GridMap, load_grid, load_trajectory,  # noqa: E402
                    parse_points, setup_japanese_font)

LABEL_BOX = dict(boxstyle="round,pad=0.3", fc="white", ec="#666", alpha=0.93)


def auto_limits(grid: GridMap, traj: np.ndarray | None, points: list[tuple[float, float]],
                margin: float) -> tuple[list[float], list[float]] | None:
    """軌跡と目的地（と原点）を囲む範囲に margin [m] を足した表示範囲。地図の外には出さない。軌跡も目的地も無ければ None。

    地図は歩いた範囲よりずっと広く写る（LiDAR は遠くまで届く）ので、全体を出すと肝心の通路が小さくなる。
    2026-10-17 の実機では x −2〜15・y −6〜2 に切り詰めた表示で G1 の動きを追った。
    """
    xs, ys = [0.0], [0.0]
    if traj is not None:
        xs += list(traj[:, 1])
        ys += list(traj[:, 2])
    xs += [p[0] for p in points]
    ys += [p[1] for p in points]
    if len(xs) == 1:
        return None
    ext = grid.extent
    return ([max(ext[0], math.floor(min(xs) - margin)), min(ext[1], math.ceil(max(xs) + margin))],
            [max(ext[2], math.floor(min(ys) - margin)), min(ext[3], math.ceil(max(ys) + margin))])


def draw_base(ax: plt.Axes, grid: GridMap, traj: np.ndarray | None, points: list[tuple[float, float]],
              loop: bool, narrow: float, xlim: list[float] | None, ylim: list[float] | None,
              minimal: bool = False) -> None:
    """地図・軌跡・原点・目的地を描く（render_map と live_map で共通の背景）。

    minimal=True は走行中の表示（live_map）用。2026-10-17 に実機で使った画像（live_position_20261017.png）と同じく、
    原点の矢印・2 m の物差し・長い軸の説明を省き、位置と通った跡を見やすくする。
    """
    ext = grid.extent
    ax.imshow(grid.rgb(narrow), extent=ext, origin="upper", interpolation="nearest")
    if traj is not None:
        ax.plot(traj[:, 1], traj[:, 2], color="#2b7bba", lw=1.0 if minimal else 1.2, alpha=0.4 if minimal else 0.6)
    ax.plot(0, 0, "o", ms=9 if minimal else 10, color="#2ca02c", mec="k", zorder=5)
    if not minimal:
        ax.annotate("", (1.0, 0), (0, 0), arrowprops=dict(arrowstyle="-|>", color="#2ca02c", lw=2.2), zorder=5)
    if points:
        xs = [p[0] for p in points] + ([points[0][0]] if loop else [])
        ys = [p[1] for p in points] + ([points[0][1]] if loop else [])
        ax.plot(xs, ys, "--", color="#ff7f0e", lw=1.5 if minimal else 1.8, alpha=0.7 if minimal else 0.85, zorder=4)
        for i, (x, y) in enumerate(points, 1):
            ax.text(x, y, str(i), ha="center", va="center", fontsize=9 if minimal else 10, weight="bold", zorder=7,
                    bbox=dict(boxstyle="circle,pad=0.2", fc="#ffc56b" if minimal else "#ffb347", ec="k"))
    ax.set_xticks(np.arange(math.floor(ext[0]), math.ceil(ext[1]) + 0.01, 1))
    ax.set_yticks(np.arange(math.floor(ext[2]), math.ceil(ext[3]) + 0.01, 1))
    ax.grid(color="#7a8699", alpha=0.35, lw=0.6)
    ax.set_xlim(*(xlim or ext[:2]))
    ax.set_ylim(*(ylim or ext[2:]))
    ax.set_aspect("equal")
    if minimal:
        ax.set_xlabel("x [m]")
        ax.set_ylabel("y [m]")
        return
    ax.set_xlabel("x [m]（地図の座標。原点で G1 が向いていた方向が +x）")
    ax.set_ylabel("y [m]（+y は左）")
    x0, x1 = ax.get_xlim()
    y0, y1 = ax.get_ylim()
    sx, sy = x1 - 0.6 - 2.0, y1 - 0.5
    ax.plot([sx, sx + 2.0], [sy, sy], color="k", lw=4, zorder=8)
    ax.text(sx + 1.0, sy + 0.15, "2 m", ha="center", fontsize=9, bbox=LABEL_BOX, zorder=8)


def legend_handles(narrow: float, has_traj: bool, has_points: bool) -> list:
    hand = [Patch(fc="white", ec="#999", label="通れる場所"),
            Patch(color=COLOR_NARROW, label=f"狭い所（障害物まで {narrow:.2f} m 未満）"),
            Patch(color=COLOR_OCCUPIED, label="障害物（机・椅子・壁など）"),
            Patch(color=COLOR_UNKNOWN, label="未知（見えていない所。通れない扱い）")]
    if has_traj:
        hand.append(Line2D([], [], color="#2b7bba", lw=1.4, label="地図を作ったときに歩いた経路"))
    if has_points:
        hand.append(Line2D([], [], color="#ff7f0e", ls="--", lw=1.8, label="目的地の順番（実際の経路は Nav2 が決める）"))
    hand.append(Line2D([], [], marker="o", ls="", color="#2ca02c", mec="k", label="原点 (0, 0) と +x の向き"))
    return hand


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, required=True, help="map_server 形式の .yaml")
    ap.add_argument("--trajectory", type=Path, help="地図を作ったときの軌跡（TUM 形式）")
    ap.add_argument("--points", type=float, nargs="+", help="目的地 x1 y1 x2 y2 ...（地図の座標 [m]）")
    ap.add_argument("--points-file", type=Path, help="目的地のファイル（1 行に x y）")
    ap.add_argument("--loop", action="store_true", help="最後の目的地から最初へ線を戻す（見回りの 1 周）")
    ap.add_argument("--pose", type=float, nargs=3, metavar=("X", "Y", "YAW_DEG"), help="G1 の現在位置と向き [度]")
    ap.add_argument("--narrow", type=float, default=0.35, help="障害物までこの距離 [m] 未満を「狭い所」にする（G1 の体の半径の目安）")
    ap.add_argument("--xlim", type=float, nargs=2)
    ap.add_argument("--ylim", type=float, nargs=2)
    ap.add_argument("--crop", type=float, default=None, metavar="MARGIN",
                    help="軌跡と目的地を囲む範囲 + MARGIN [m] に切り詰める（例 2）。既定は地図全体")
    ap.add_argument("--title", default=None)
    ap.add_argument("--dpi", type=int, default=110)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    font = setup_japanese_font()
    if font is None:
        print("[render_map] 日本語のフォントが見つからない。文字が化ける場合は fonts-noto-cjk を入れる")
    grid = load_grid(args.map)
    traj = load_trajectory(args.trajectory)
    points = parse_points(args.points, args.points_file)

    xlim, ylim = args.xlim, args.ylim
    if args.crop is not None and not (xlim and ylim):
        lim = auto_limits(grid, traj, points, args.crop)
        if lim:
            xlim, ylim = xlim or lim[0], ylim or lim[1]
    fig, ax = plt.subplots(figsize=(16, 9.5))
    draw_base(ax, grid, traj, points, args.loop, args.narrow, xlim, ylim)
    if args.pose:
        x, y, yaw = args.pose[0], args.pose[1], math.radians(args.pose[2])
        ax.plot(x, y, "o", ms=14, color="#d62728", mec="k", zorder=9)
        ax.annotate("", (x + math.cos(yaw), y + math.sin(yaw)), (x, y),
                    arrowprops=dict(arrowstyle="-|>", color="#a51d1d", lw=2.5), zorder=9)
        ax.annotate(f"G1 ({x:+.2f}, {y:+.2f})  向き {args.pose[2]:+.0f}°", (x, y), (x + 0.4, y - 1.2),
                    fontsize=11, color="#a51d1d", weight="bold", bbox=LABEL_BOX, zorder=10,
                    arrowprops=dict(arrowstyle="-", color="#a51d1d", lw=0.9))
    for i, (px, py) in enumerate(points, 1):
        if not grid.is_free(px, py):
            print(f"[render_map] ⚠ 目的地 {i} ({px}, {py}) は通れる場所ではない")
        elif grid.clearance_at(px, py) < args.narrow:
            print(f"[render_map] ⚠ 目的地 {i} ({px}, {py}) は障害物まで {grid.clearance_at(px, py):.2f} m（狭い）")
    ax.legend(handles=legend_handles(args.narrow, traj is not None, bool(points)), loc="lower right", fontsize=10,
              framealpha=0.95)
    res_cm = grid.resolution * 100
    ax.set_title(args.title or f"G1 の地図 {grid.name}（{res_cm:g} cm/マス）", fontsize=15, weight="bold")
    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=args.dpi)
    print(f"[render_map] → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
