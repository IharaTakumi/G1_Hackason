"""Nav2 で G1 を自律移動させている間、G1 の位置を地図の画像に逐次描き続ける（操作 PC で動かす。**見るだけ**）。

PC2 の map_web.py（`Navigation/nav2_stable/tools/map_web.py`。PC2 の中で TF map→base_link を読み、
`http://<PC2>:8090/state.json` で配る）から --interval 秒ごとに位置を受け取り、PNG を描き直す。
VSCode などでこの PNG を開いておくと、書き換わるたびに表示が更新される（書き終えてから置き換えるので、途中の画像は見えない）。
無線では DDS が届かないので、ROS の購読は PC2 の中で済ませ、無線には HTTP だけを流す（WIRELESS.md の方針と同じ）。

    python3 live_map.py --map navmap.yaml --trajectory trajectory.tum --points-file patrol.txt --loop \\
        --host 192.168.0.82 --out live_position.png
    python3 live_map.py --map navmap.yaml --state-file nav_state.json --once --out check.png   # 保存した state で 1 枚だけ

描くもの: 現在位置（赤い点）と向き（矢印）、通った跡（赤い線）、Nav2 の計画経路（緑の線）、目的地（番号付き）、
状態（READY / NAVIGATING / FAULT と理由）、位置の古さ。位置が --stale 秒以上更新されないと点を灰色にして警告を出す。
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import urllib.request
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
from maplib import load_grid, load_trajectory, parse_points, setup_japanese_font  # noqa: E402
from render_map import LABEL_BOX, auto_limits, draw_base  # noqa: E402


def fetch_state(url: str) -> dict | None:
    try:
        with urllib.request.urlopen(url, timeout=2.0) as r:
            return json.loads(r.read().decode("utf-8"))
    except Exception:  # noqa: BLE001 — 無線が一時的に切れても描き続ける
        return None


def bridge_text(st: dict) -> str:
    b = st.get("bridge")
    if isinstance(b, dict):
        s = str(b.get("state") or b.get("message") or "-")
        return s + (f"（{b['fault_reason']}）" if b.get("fault_reason") else "")
    return str(b) if b else "-"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--map", type=Path, required=True, help="map_server 形式の .yaml（Nav2 に渡したものと同じ地図）")
    ap.add_argument("--trajectory", type=Path, help="地図を作ったときの軌跡（TUM 形式）")
    ap.add_argument("--points", type=float, nargs="+", help="目的地 x1 y1 x2 y2 ...")
    ap.add_argument("--points-file", type=Path, help="目的地のファイル（1 行に x y）")
    ap.add_argument("--loop", action="store_true", help="最後の目的地から最初へ線を戻す")
    ap.add_argument("--reverse", action="store_true", help="目的地の番号を逆回りの順に付ける")
    ap.add_argument("--host", default="192.168.0.82", help="PC2 の IP（無線なら無線側）")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--state-file", type=Path, help="HTTP の代わりに、保存した state.json を読む（確認用）")
    ap.add_argument("--interval", type=float, default=1.0, help="描き直す間隔 [s]")
    ap.add_argument("--stale", type=float, default=2.0, help="位置がこの秒数以上古ければ警告する")
    ap.add_argument("--narrow", type=float, default=0.35)
    ap.add_argument("--xlim", type=float, nargs=2)
    ap.add_argument("--ylim", type=float, nargs=2)
    ap.add_argument("--crop", type=float, default=1.5, metavar="MARGIN",
                    help="軌跡と目的地を囲む範囲 + MARGIN [m] に切り詰める（既定 1.5。負の値で地図全体）")
    ap.add_argument("--save-state", type=Path, default=None,
                    help="受け取った最新の state.json をここに毎回保存する（止まったあとに状況を振り返るため）")
    ap.add_argument("--once", action="store_true", help="1 枚だけ描いて終わる")
    ap.add_argument("--dpi", type=int, default=90)
    ap.add_argument("--out", type=Path, required=True)
    args = ap.parse_args()

    setup_japanese_font()
    grid = load_grid(args.map)
    traj = load_trajectory(args.trajectory)
    points = parse_points(args.points, args.points_file)
    if args.reverse:
        points.reverse()

    xlim, ylim = args.xlim, args.ylim
    if args.crop >= 0 and not (xlim and ylim):
        lim = auto_limits(grid, traj, points, args.crop)
        if lim:
            xlim, ylim = xlim or lim[0], ylim or lim[1]
    fig, ax = plt.subplots(figsize=(15, 8))
    draw_base(ax, grid, traj, points, args.loop, args.narrow, xlim, ylim, minimal=True)
    # 配置（tight_layout）を決める前に、同じ長さの題名を入れておく（空のままだと上が切れる）
    title = ax.set_title("G1 の現在地（赤い点・矢印）  通った跡＝赤い線  計画経路＝緑の線   00:00:00",
                         fontsize=14, weight="bold")
    (trail_line,) = ax.plot([], [], "-", color="#d62728", lw=2.2, alpha=0.85, zorder=6)
    (plan_line,) = ax.plot([], [], "-", color="#2ca02c", lw=2.0, alpha=0.9, zorder=6)
    (dot,) = ax.plot([], [], "o", ms=16, color="#d62728", mec="k", mew=1.5, zorder=9)
    info = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", fontsize=12, bbox=LABEL_BOX, zorder=10)
    arrow = None
    fig.tight_layout()

    url = f"http://{args.host}:{args.port}/state.json"
    src = str(args.state_file) if args.state_file else url
    args.out.parent.mkdir(parents=True, exist_ok=True)
    tmp = args.out.with_name(args.out.stem + ".tmp" + args.out.suffix)
    print(f"[live_map] {src} → {args.out}（{'1 枚' if args.once else f'{args.interval:.1f} 秒ごと。Ctrl-C で終わる'}）", flush=True)
    try:
        while True:
            t0 = time.time()
            st = json.loads(args.state_file.read_text()) if args.state_file else fetch_state(url)
            now = time.strftime("%H:%M:%S")
            if arrow is not None:
                arrow.remove()
                arrow = None
            if st is None:
                # 最後に分かっていた位置と通った跡は残し、点を灰色にする（止まった場所が消えないように）
                info.set_text(f"{now}  PC2 の map_web.py に届かない（{args.host}:{args.port}）")
                dot.set_color("#999999")
            else:
                if args.save_state and not args.state_file:
                    tmp_state = args.save_state.with_name(args.save_state.name + ".tmp")
                    tmp_state.write_text(json.dumps(st, ensure_ascii=False))
                    os.replace(tmp_state, args.save_state)
                pose, age = st.get("pose"), st.get("pose_age_s")
                trail, plan = st.get("trail") or [], st.get("plan") or []
                trail_line.set_data([p[0] for p in trail], [p[1] for p in trail])
                plan_line.set_data([p[0] for p in plan], [p[1] for p in plan])
                stale = age is None or age > args.stale
                if pose:
                    x, y, yaw = pose["x"], pose["y"], pose["yaw"]
                    dot.set_data([x], [y])
                    dot.set_color("#999999" if stale else "#d62728")
                    arrow = ax.annotate("", (x + 0.8 * math.cos(yaw), y + 0.8 * math.sin(yaw)), (x, y),
                                        arrowprops=dict(arrowstyle="-|>", color="#7a0f10", lw=2.5), zorder=10)
                    pos = f"位置 ({x:+.2f}, {y:+.2f})  向き {math.degrees(yaw):+.0f}°"
                else:
                    pos = "位置 まだ無い"
                warn = "  ⚠ 位置が更新されていない" if stale else ""
                age_s = "-" if age is None else f"{age:.1f} 秒前"
                info.set_text(f"{now}  状態 {bridge_text(st)}\n{pos}\n位置の更新 {age_s}{warn}")
            title.set_text(f"G1 の現在地（赤い点・矢印）  通った跡＝赤い線  計画経路＝緑の線   {now}")
            fig.savefig(tmp, dpi=args.dpi)
            os.replace(tmp, args.out)
            if args.once:
                break
            time.sleep(max(0.0, args.interval - (time.time() - t0)))
    except KeyboardInterrupt:
        print("[live_map] 終了")
    print(f"[live_map] → {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
