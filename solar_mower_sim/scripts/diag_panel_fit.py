#!/usr/bin/env python3
"""LiDAR の直線抽出 (パネル架台の横梁) を診断する。

/scan を受け取り、左右それぞれの点数・RANSAC の結果 (距離・角度・インライア数) を表示する。
壁追従が "panel lost" になる、振動する等の切り分け用。

usage (シミュレーション起動中に): ./sim python3 /ws/src/solar_mower_sim/scripts/diag_panel_fit.py [回数=5]
"""
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from solar_mower_sim.controllers import LineFitter  # noqa: E402

COUNT = int(sys.argv[1]) if len(sys.argv) > 1 else 5


def describe(scan: LaserScan) -> None:
    r = np.asarray(scan.ranges, dtype=float)
    finite = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
    print(f"scan: {len(r)} rays, angle {math.degrees(scan.angle_min):.0f}..{math.degrees(scan.angle_max):.0f} deg, "
          f"range {scan.range_min:.2f}..{scan.range_max:.1f} m, valid {int(finite.sum())}")
    ang = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    for lo in range(-180, 180, 45):
        m = finite & (np.degrees(ang) >= lo) & (np.degrees(ang) < lo + 45)
        print(f"  {lo:4d}..{lo + 45:4d} deg: {int(m.sum()):3d} hits, min {r[m].min() if m.any() else float('nan'):.2f} m")
    fitter = LineFitter()
    for side, name in ((1, "left"), (-1, "right")):
        pts = LineFitter.scan_to_points(r, scan.angle_min, scan.angle_increment,
                                        scan.range_min, scan.range_max, side=side)
        fit = fitter.fit(pts)
        if fit is None:
            print(f"  {name:5s}: {len(pts):3d} pts within 4 m -> no line")
        else:
            print(f"  {name:5s}: {len(pts):3d} pts -> dist {fit.distance:.3f} m, angle {math.degrees(fit.angle):+.1f} deg, "
                  f"side {fit.side:+d}, inliers {fit.inliers}/{fit.total}")


def main() -> None:
    rclpy.init()
    node = rclpy.create_node("diag_panel_fit")
    seen = []

    def cb(msg: LaserScan) -> None:
        seen.append(1)
        print(f"--- #{len(seen)}")
        describe(msg)

    node.create_subscription(LaserScan, "scan", cb, qos_profile_sensor_data)
    while rclpy.ok() and len(seen) < COUNT:
        rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
