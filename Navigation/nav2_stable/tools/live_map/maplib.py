"""G1 の Nav2 用の地図（map_server 形式の .yaml + .pgm）を、人が見やすい画像にするための共通部品。

render_map.py / live_map.py / pick_waypoints.py から使う。numpy・opencv・scipy・matplotlib だけで動く（ROS 不要）。
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage

# 色（RGB, 0〜1）。凡例の文言と対で使う
COLOR_FREE = (1.0, 1.0, 1.0)
COLOR_UNKNOWN = (0.82, 0.84, 0.88)
COLOR_OCCUPIED = (0.15, 0.17, 0.22)
COLOR_NARROW = (1.0, 0.86, 0.70)


@dataclass
class GridMap:
    """map_server 形式の地図。cls は 0=通れる 1=未知 2=障害物、clearance は最も近い障害物までの距離 [m]。"""

    cls: np.ndarray
    clearance: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float
    name: str

    @property
    def extent(self) -> list[float]:
        """imshow の extent（[x0, x1, y0, y1]、地図の座標 [m]）。"""
        h, w = self.cls.shape
        return [self.origin_x, self.origin_x + w * self.resolution, self.origin_y, self.origin_y + h * self.resolution]

    def cell(self, x: float, y: float) -> tuple[int, int] | None:
        """地図の座標 → 画像の (行, 列)。範囲外なら None。"""
        h, w = self.cls.shape
        r = h - 1 - int((y - self.origin_y) / self.resolution)
        c = int((x - self.origin_x) / self.resolution)
        return (r, c) if 0 <= r < h and 0 <= c < w else None

    def xy(self, r: int, c: int) -> tuple[float, float]:
        """画像の (行, 列) → 地図の座標（セルの中心）。"""
        h = self.cls.shape[0]
        return (self.origin_x + (c + 0.5) * self.resolution, self.origin_y + (h - 1 - r + 0.5) * self.resolution)

    def is_free(self, x: float, y: float) -> bool:
        rc = self.cell(x, y)
        return rc is not None and self.cls[rc] == 0

    def clearance_at(self, x: float, y: float) -> float:
        rc = self.cell(x, y)
        return float(self.clearance[rc]) if rc is not None else 0.0

    def rgb(self, narrow_m: float) -> np.ndarray:
        """色分けした画像（H, W, 3）。通れるが障害物まで narrow_m 未満のセルを「狭い所」の色にする。"""
        img = np.zeros((*self.cls.shape, 3))
        img[self.cls == 0] = COLOR_FREE
        img[self.cls == 1] = COLOR_UNKNOWN
        img[self.cls == 2] = COLOR_OCCUPIED
        if narrow_m > 0:
            img[(self.cls == 0) & (self.clearance < narrow_m)] = COLOR_NARROW
        return img


def _yaml_value(text: str, key: str) -> str | None:
    m = re.search(rf"^\s*{key}\s*:\s*(.+?)\s*$", text, re.MULTILINE)
    return m.group(1) if m else None


def load_grid(yaml_path: Path) -> GridMap:
    """map_server 形式の .yaml を読む（PyYAML 不要。image / resolution / origin / negate / *_thresh だけ使う）。"""
    text = yaml_path.read_text()
    image = _yaml_value(text, "image")
    if image is None:
        raise SystemExit(f"[maplib] image が無い: {yaml_path}")
    img_path = Path(image.strip("'\"")) if Path(image.strip("'\"")).is_absolute() else yaml_path.parent / image.strip("'\"")
    pix = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    if pix is None:
        raise SystemExit(f"[maplib] 画像が読めない: {img_path}")
    res = float(_yaml_value(text, "resolution") or 0.05)
    origin = [float(v) for v in re.findall(r"[-\d.eE+]+", _yaml_value(text, "origin") or "[0, 0, 0]")][:2]
    negate = int(float(_yaml_value(text, "negate") or 0))
    occ_th = float(_yaml_value(text, "occupied_thresh") or 0.65)
    free_th = float(_yaml_value(text, "free_thresh") or 0.196)
    # map_server と同じ判定: 占有確率 p = (255 - 画素) / 255（negate なら 画素 / 255）
    p = (pix.astype(float) / 255.0) if negate else (255.0 - pix.astype(float)) / 255.0
    cls = np.ones(pix.shape, np.uint8)
    cls[p < free_th] = 0
    cls[p > occ_th] = 2
    clearance = ndimage.distance_transform_edt(cls != 2) * res
    return GridMap(cls, clearance, res, origin[0], origin[1], yaml_path.stem)


def load_trajectory(path: Path | None) -> np.ndarray | None:
    """TUM 形式（t x y z qx qy qz qw）の軌跡。無ければ None。"""
    if path is None:
        return None
    data = np.loadtxt(path, ndmin=2)
    return data if len(data) else None


def parse_points(values: list[float] | None, path: Path | None) -> list[tuple[float, float]]:
    """目的地の列。--points x1 y1 x2 y2 ... か、1 行に `x y`（カンマ区切りも可）のファイル。"""
    pts: list[tuple[float, float]] = []
    if path is not None:
        for line in path.read_text().splitlines():
            nums = re.findall(r"[-\d.eE+]+", line.split("#")[0])
            if len(nums) >= 2:
                pts.append((float(nums[0]), float(nums[1])))
    if values:
        if len(values) % 2:
            raise SystemExit("[maplib] --points は x y の組で渡す")
        pts += [(values[i], values[i + 1]) for i in range(0, len(values), 2)]
    return pts


def setup_japanese_font() -> str | None:
    """matplotlib で日本語を出すためのフォントを選ぶ（Noto Sans CJK JP など）。見つからなければ None。"""
    from matplotlib import font_manager
    import matplotlib.pyplot as plt

    for f in font_manager.findSystemFonts():
        if "CJK" in f or "Noto" in f and "JP" in f:
            try:
                font_manager.fontManager.addfont(f)
            except Exception:  # noqa: BLE001 — 読めないフォントは飛ばす
                pass
    names = {f.name for f in font_manager.fontManager.ttflist}
    for cand in ("Noto Sans CJK JP", "Noto Sans JP", "IPAexGothic", "IPAGothic", "TakaoGothic", "Noto Serif CJK JP"):
        if cand in names:
            plt.rcParams["font.family"] = cand
            return cand
    return None


def yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
