"""レーン (パネル列の間) を往復して刈る経路計画 (ROS 非依存、単体テスト可能)。

レーンの左端から刈り幅ずつずらしながら往復し、右端まで刈る:

    北の列 ══════════════════════
      ①→→→→→→→→→→→→→→→→→→→→┐   各パスは近い方の列を基準に follow_panel
      ┌←←←←←←←←←←←←←←←←←←←←②┘   端で 90°旋回 → 横に swath 進む → 90°旋回
      └③→→→→→→→→→→→→→→→→→→→→┐
         ...
    南の列 ══════════════════════

すべて既存のアクション (follow_panel / turn / go_straight) の列に展開するので、
executor の状態機械はそのまま使える。
"""
from __future__ import annotations

import math
from typing import Any, Dict, List


def lane_offsets(width: float, margin: float, swath: float) -> List[float]:
    """左の列 (の横梁の外面) からの距離のリスト。左端 margin から右端 width - margin まで等間隔。
    間隔は swath 以下になるように本数を決める (刈り残しを作らない)。"""
    span = width - 2 * margin
    if span <= 0:
        return [width / 2]
    n = math.ceil(span / swath - 1e-9) + 1
    return [margin + span * i / (n - 1) for i in range(n)]


def plan_mow_lane(left_dist: float, right_dist: float, length: float,
                  swath: float = 0.6, margin: float = 0.6, speed: float = 0.5,
                  start_side: str = "left") -> List[Dict[str, Any]]:
    """現在位置 (レーン内、レーンの向きに向いている) からの往復草刈りの手順を作る。

    left_dist / right_dist: LiDAR で測った左右の列までの距離 [m]
    length: 1 パスで進む距離 [m]
    start_side: "left" なら左端から右端へ、"right" なら右端から左端へ
    """
    width = left_dist + right_dist
    sgn = 1 if start_side == "left" else -1          # 進む向き: left 開始なら右へずれていく
    # 開始側の列からの距離で考える
    near_now = left_dist if sgn > 0 else right_dist
    offsets = lane_offsets(width, margin, swath)
    steps: List[Dict[str, Any]] = []

    # 1 本目のレーンへ横移動 (開始側へ near_now - offsets[0])
    move = near_now - offsets[0]
    if abs(move) > 0.1:
        toward_start = 90.0 * sgn if move > 0 else -90.0 * sgn   # left 開始なら左 (+90)
        steps += [{"action": "turn", "angle_deg": toward_start},
                  {"action": "go_straight", "distance": round(abs(move), 3), "speed": 0.3},
                  {"action": "turn", "angle_deg": -toward_start}]

    for i, d in enumerate(offsets):
        forward = i % 2 == 0                          # 偶数パス: 最初の向き、奇数パス: 逆向き
        # 近い方の列を基準にする。開始側の列は、往路では start_side、復路では反対側に見える
        if d <= width / 2:
            ref_is_start_row, dist = True, d
        else:
            ref_is_start_row, dist = False, width - d
        start_row_side = start_side if forward else ("right" if start_side == "left" else "left")
        if ref_is_start_row:
            side = start_row_side
        else:
            side = "right" if start_row_side == "left" else "left"
        steps.append({"action": "follow_panel", "side": side, "distance": round(dist, 3),
                      "speed": speed, "length": length})
        if i + 1 < len(offsets):
            # 隣のレーンへ: 開始側から離れる向きに 90°、swath 進んで、さらに 90° (= Uターン)
            away = -90.0 * sgn if forward else 90.0 * sgn
            steps += [{"action": "turn", "angle_deg": away},
                      {"action": "go_straight", "distance": round(offsets[i + 1] - d, 3), "speed": 0.3},
                      {"action": "turn", "angle_deg": away}]
    return steps
