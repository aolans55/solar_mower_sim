#!/usr/bin/env python3
"""ソーラーパネル列 (panel_row) の SDF モデルを生成する。

1 列 = 2 m 幅のパネルを x 方向に N 枚並べたもの。
  - パネル: 2.0 x 1.0 m, 南向き (−y) に 20° 傾斜, 下端高さ 0.70 m
  - 支柱  : 2 m ごとに 8 cm 角, 地面〜架台
  - 横梁  : 列の前面 (−y 側) に高さ 0.55 m で連続 → LiDAR が捉える基準線
生成物は models/panel_row/model.sdf に書き出す。
"""
import math
import os
import sys

N_PANELS = int(sys.argv[1]) if len(sys.argv) > 1 else 6
PANEL_W = 2.0      # x 方向幅
PANEL_D = 1.0      # 傾斜方向の長さ
PANEL_T = 0.04
TILT = math.radians(20.0)
LOWER_EDGE_Z = 0.70
BEAM_Z = 0.55
BEAM_SIZE = 0.10
POST_SIZE = 0.08

row_len = N_PANELS * PANEL_W
# パネル中心 (傾斜を考慮)
pc_y = -PANEL_D / 2 * math.cos(TILT) + PANEL_D / 2 * math.cos(TILT)  # =0 → 中心を y=0 に置く
pc_z = LOWER_EDGE_Z + PANEL_D / 2 * math.sin(TILT)
front_y = -PANEL_D / 2 * math.cos(TILT)   # 下端 (南側) の y

def box(name, pose, size, rgba, collide=True):
    s = f"""      <visual name="{name}_v">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material>
      </visual>
"""
    if collide:
        s += f"""      <collision name="{name}_c">
        <pose>{pose}</pose>
        <geometry><box><size>{size}</size></box></geometry>
      </collision>
"""
    return s

body = []
for i in range(N_PANELS):
    x = -row_len / 2 + PANEL_W / 2 + i * PANEL_W
    body.append(box(f"panel_{i}", f"{x:.3f} 0 {pc_z:.3f} {TILT:.4f} 0 0",
                    f"{PANEL_W} {PANEL_D} {PANEL_T}", "0.05 0.10 0.35 1"))
# 支柱 (列の両端と各パネル境界)
for i in range(N_PANELS + 1):
    x = -row_len / 2 + i * PANEL_W
    post_h = LOWER_EDGE_Z + PANEL_D / 2 * math.sin(TILT)
    body.append(box(f"post_{i}", f"{x:.3f} 0 {post_h/2:.3f} 0 0 0",
                    f"{POST_SIZE} {POST_SIZE} {post_h:.3f}", "0.6 0.6 0.62 1"))
# 前面横梁 (LiDAR 基準線)
body.append(box("front_beam", f"0 {front_y:.3f} {BEAM_Z:.3f} 0 0 0",
                f"{row_len} {BEAM_SIZE} {BEAM_SIZE}", "0.55 0.55 0.58 1"))
# 背面横梁 (見た目)
body.append(box("rear_beam", f"0 {-front_y:.3f} {BEAM_Z + PANEL_D*math.sin(TILT):.3f} 0 0 0",
                f"{row_len} {BEAM_SIZE} {BEAM_SIZE}", "0.55 0.55 0.58 1"))

sdf = f"""<?xml version="1.0" ?>
<!-- 自動生成: scripts/gen_panel_row.py {N_PANELS} -->
<sdf version="1.9">
  <model name="panel_row">
    <static>true</static>
    <link name="row_link">
{''.join(body)}    </link>
  </model>
</sdf>
"""

out = os.path.join(os.path.dirname(__file__), "..", "models", "panel_row", "model.sdf")
with open(out, "w") as f:
    f.write(sdf)
print(f"wrote {os.path.abspath(out)} ({N_PANELS} panels, row length {row_len} m, front beam y={front_y:.2f})")
