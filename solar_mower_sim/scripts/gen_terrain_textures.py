#!/usr/bin/env python3
"""不整地 heightmap 用のテクスチャ (草・土・平坦法線) を生成する。

Gazebo Classic の file://media/materials/textures/... は Gazebo Harmonic に無いので自前で用意する。
FFT でぼかしたノイズは周期的なので、繰り返し貼ってもつなぎ目が出ない。
生成物は models/solar_farm_terrain/materials/textures/ に書き出す。
"""
import os
import sys

import numpy as np

try:
    from PIL import Image
except ImportError:
    sys.exit("pip install pillow が必要です")

N = 512
SEED = int(sys.argv[1]) if len(sys.argv) > 1 else 7
rng = np.random.default_rng(SEED)
OUT = os.path.join(os.path.dirname(__file__), "..", "models", "solar_farm_terrain",
                   "materials", "textures")


def tileable_noise(sigma: float) -> np.ndarray:
    """周期境界のガウスぼかしノイズ (0..1)"""
    f = np.fft.fftfreq(N)
    k2 = f[:, None] ** 2 + f[None, :] ** 2
    g = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((N, N)))
                             * np.exp(-2 * (np.pi * sigma) ** 2 * k2)))
    g -= g.min()
    return g / g.max()


def colorize(base, dark, light, octaves) -> np.ndarray:
    n = sum(w * tileable_noise(s) for s, w in octaves) / sum(w for _, w in octaves)
    n = np.clip((n - 0.5) * 1.8 + 0.5, 0, 1)[..., None]
    rgb = np.asarray(dark) * (1 - n) + np.asarray(light) * n
    rgb = rgb * 0.85 + np.asarray(base) * 0.15
    return np.clip(rgb, 0, 255).astype(np.uint8)


os.makedirs(OUT, exist_ok=True)
grass = colorize((90, 130, 50), (55, 95, 35), (125, 165, 70), [(1.0, 0.6), (4.0, 0.3), (16.0, 0.1)])
dirt = colorize((120, 95, 65), (90, 70, 50), (150, 125, 90), [(1.5, 0.5), (6.0, 0.3), (24.0, 0.2)])
flat = np.zeros((8, 8, 3), np.uint8) + np.array([128, 128, 255], np.uint8)
for name, img in (("grass_diffuse.png", grass), ("dirt_diffuse.png", dirt), ("flat_normal.png", flat)):
    path = os.path.join(OUT, name)
    Image.fromarray(img).save(path)
    print(f"wrote {os.path.abspath(path)}  shape={img.shape}")
