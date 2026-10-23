#!/usr/bin/env python3
"""不整地 heightmap (PNG, 16bit グレースケール) を生成する。

Gazebo の heightmap は (2^n + 1) 正方の画像が必要 → 129 x 129 を使用。
world 側で <size>40 40 0.35</size> とすると最大高低差 0.35 m 程度の凸凹になる。

地形の成分:
  - 低周波のうねり (畑の起伏)
  - 中周波のランダムバンプ (草の株・轍)
  - パネル列の直下 (y = ±4 m 付近) はやや平坦に
"""
import os
import sys
import numpy as np

try:
    from PIL import Image
except ImportError:
    sys.exit("pip install pillow が必要です")

N = 129
SEED = int(sys.argv[1]) if len(sys.argv) > 1 else 42
rng = np.random.default_rng(SEED)

y, x = np.mgrid[0:N, 0:N] / (N - 1)  # 0..1

# 低周波うねり
h = (0.35 * np.sin(2 * np.pi * (1.3 * x + 0.2))
     + 0.30 * np.sin(2 * np.pi * (1.9 * y + 0.7))
     + 0.25 * np.sin(2 * np.pi * (2.4 * x + 1.7 * y)))

# 中周波バンプ: ランダムなガウス山
for _ in range(60):
    cx, cy = rng.random(2)
    sig = rng.uniform(0.015, 0.05)
    amp = rng.uniform(-0.6, 0.9)
    h += amp * np.exp(-((x - cx) ** 2 + (y - cy) ** 2) / (2 * sig ** 2))

# 細かいノイズ
h += 0.08 * rng.standard_normal((N, N))

# パネル列直下は平坦寄り (列はワールド y = ±ROW_Y。地形は 40 m 四方)
ROW_Y, SIZE = 2.42, 40.0
for yc in (0.5 - ROW_Y / SIZE, 0.5 + ROW_Y / SIZE):
    w = np.exp(-((y - yc) ** 2) / (2 * 0.025 ** 2))
    h = h * (1 - 0.6 * w)

h -= h.min()
h /= h.max()
img = (h * 65535).astype(np.uint16)

out = os.path.join(os.path.dirname(__file__), "..", "models", "solar_farm_terrain", "heightmap.png")
Image.fromarray(img).save(out)
print(f"wrote {os.path.abspath(out)}  shape={img.shape} seed={SEED}")
