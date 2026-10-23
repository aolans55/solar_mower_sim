"""ROS 非依存の制御ロジック (単体テスト可能)。

- LineFitter          : LiDAR 点群から RANSAC でパネル架台の直線を抽出
- PanelFollowController: 直線との距離・角度を一定に保つ PID (壁追従)
- HeadingController   : IMU ヨー角を目標方位に保持する PID (不整地直進)
- PID                 : 汎用 PID

座標系はすべてロボット base_link 基準 (+x 前, +y 左, 角度は反時計回り正)。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import numpy as np


def wrap_angle(a: float) -> float:
    """角度を (-pi, pi] に正規化"""
    return math.atan2(math.sin(a), math.cos(a))


# ----------------------------------------------------------------------------
@dataclass
class PID:
    kp: float
    ki: float = 0.0
    kd: float = 0.0
    out_limit: float = 1.0
    i_limit: float = 0.5
    _i: float = field(default=0.0, init=False)
    _prev_e: Optional[float] = field(default=None, init=False)

    def reset(self) -> None:
        self._i = 0.0
        self._prev_e = None

    def step(self, error: float, dt: float) -> float:
        if dt <= 0:
            dt = 1e-3
        self._i = max(-self.i_limit, min(self.i_limit, self._i + error * dt))
        d = 0.0 if self._prev_e is None else (error - self._prev_e) / dt
        self._prev_e = error
        u = self.kp * error + self.ki * self._i + self.kd * d
        return max(-self.out_limit, min(self.out_limit, u))


# ----------------------------------------------------------------------------
@dataclass
class LineFit:
    """ロボット座標系での直線 n·p = d  (n: 単位法線, d: 原点からの符号付き距離)"""
    nx: float
    ny: float
    d: float
    inliers: int
    total: int

    @property
    def distance(self) -> float:
        """ロボット原点から直線までの距離 (正)"""
        return abs(self.d)

    @property
    def angle(self) -> float:
        """直線の方向と +x 軸のなす角 [rad]。0 なら直線は x 軸に平行。
        方向ベクトルは法線を 90° 回したもの。符号は (-pi/2, pi/2] に正規化。"""
        a = math.atan2(self.nx, -self.ny)  # direction = (-ny, nx)
        if a > math.pi / 2:
            a -= math.pi
        elif a <= -math.pi / 2:
            a += math.pi
        return a

    @property
    def side(self) -> int:
        """直線がロボットの左(+1)か右(-1)か"""
        # 原点から直線へ下ろした足: foot = d * n
        return 1 if self.d * self.ny >= 0 else -1


class LineFitter:
    """2D 点群から RANSAC で最も支持の多い直線を抽出"""

    def __init__(self, inlier_thresh: float = 0.06, iters: int = 80,
                 min_inliers: int = 12, seed: int = 0):
        self.thresh = inlier_thresh
        self.iters = iters
        self.min_inliers = min_inliers
        self.rng = np.random.default_rng(seed)

    @staticmethod
    def scan_to_points(ranges: Sequence[float], angle_min: float, angle_inc: float,
                       range_min: float, range_max: float,
                       side: Optional[int] = None, max_dist: float = 4.0) -> np.ndarray:
        """LaserScan → (N,2) 点群。side=+1 で左半面, -1 で右半面のみ。"""
        r = np.asarray(ranges, dtype=float)
        ang = angle_min + np.arange(len(r)) * angle_inc
        ok = np.isfinite(r) & (r > range_min) & (r < min(range_max, max_dist))
        if side is not None:
            # 左半面: 角度 +15°..+165°, 右半面: -165°..-15° (真正面/真後ろは除外)
            a = np.arctan2(np.sin(ang), np.cos(ang))
            if side > 0:
                ok &= (a > math.radians(15)) & (a < math.radians(165))
            else:
                ok &= (a < math.radians(-15)) & (a > math.radians(-165))
        pts = np.stack([r[ok] * np.cos(ang[ok]), r[ok] * np.sin(ang[ok])], axis=1)
        return pts

    def fit(self, pts: np.ndarray) -> Optional[LineFit]:
        n = len(pts)
        if n < self.min_inliers:
            return None
        best_mask = None
        best_cnt = 0
        for _ in range(self.iters):
            i, j = self.rng.choice(n, 2, replace=False)
            p, q = pts[i], pts[j]
            v = q - p
            L = np.hypot(*v)
            if L < 1e-3:
                continue
            nrm = np.array([-v[1], v[0]]) / L
            d = nrm @ p
            dist = np.abs(pts @ nrm - d)
            mask = dist < self.thresh
            cnt = int(mask.sum())
            if cnt > best_cnt:
                best_cnt, best_mask = cnt, mask
        if best_mask is None or best_cnt < self.min_inliers:
            return None
        # インライアで最小二乗 (PCA) 再フィット
        P = pts[best_mask]
        c = P.mean(axis=0)
        u, s, vt = np.linalg.svd(P - c, full_matrices=False)
        direction = vt[0]
        nrm = np.array([-direction[1], direction[0]])
        d = float(nrm @ c)
        # 法線の向きを「原点→直線」方向に揃える (d >= 0)
        if d < 0:
            nrm, d = -nrm, -d
        return LineFit(float(nrm[0]), float(nrm[1]), d, best_cnt, n)


# ----------------------------------------------------------------------------
class PanelFollowController:
    """パネル架台の直線に平行、かつ一定距離で走る壁追従制御。

    角速度 = Kθ·(角度誤差) + Kd·(距離誤差)  の 2 項合成 (符号は追従側で反転)。
    """

    def __init__(self, target_dist: float = 0.9, side: int = 1,
                 v_nominal: float = 0.5,
                 kp_dist: float = 1.2, ki_dist: float = 0.05, kd_dist: float = 0.4,
                 kp_ang: float = 2.0,
                 w_limit: float = 0.8, lost_timeout: float = 1.5):
        self.target_dist = target_dist
        self.side = 1 if side >= 0 else -1
        self.v_nominal = v_nominal
        self.pid_dist = PID(kp_dist, ki_dist, kd_dist, out_limit=w_limit)
        self.kp_ang = kp_ang
        self.w_limit = w_limit
        self.lost_timeout = lost_timeout
        self._lost_time = 0.0
        self.last_fit: Optional[LineFit] = None

    def reset(self) -> None:
        self.pid_dist.reset()
        self._lost_time = 0.0
        self.last_fit = None

    def step(self, fit: Optional[LineFit], dt: float) -> Tuple[float, float, str]:
        """→ (v, w, status)   status: 'ok' | 'lost' | 'stopped'"""
        if fit is None or fit.side != self.side:
            self._lost_time += dt
            if self._lost_time > self.lost_timeout:
                return 0.0, 0.0, "stopped"
            # 直前の推定で惰性走行 (ゆっくり)
            return self.v_nominal * 0.4, 0.0, "lost"
        self._lost_time = 0.0
        self.last_fit = fit

        # 距離誤差: 遠い(+) → 壁側へ曲がる
        e_dist = fit.distance - self.target_dist
        # 角度誤差: 直線の向き (0 なら平行)。壁に向かう/離れる方向の成分
        e_ang = fit.angle

        w_dist = self.pid_dist.step(e_dist, dt) * self.side   # 左追従: 遠ければ左(+)
        w_ang = self.kp_ang * e_ang                             # 直線に平行へ
        w = max(-self.w_limit, min(self.w_limit, w_dist + w_ang))
        # 角度誤差が大きいときは減速
        v = self.v_nominal * max(0.3, 1.0 - abs(e_ang) / math.radians(30))
        return v, w, "ok"


# ----------------------------------------------------------------------------
class HeadingController:
    """ヨー角を目標方位に保持 (履帯のスリップ・凸凹での進路ずれを補正)"""

    def __init__(self, v_nominal: float = 0.5, kp: float = 2.5, ki: float = 0.1,
                 kd: float = 0.3, w_limit: float = 1.0):
        self.v_nominal = v_nominal
        self.pid = PID(kp, ki, kd, out_limit=w_limit)
        self.target_yaw: Optional[float] = None

    def set_target(self, yaw: float) -> None:
        self.target_yaw = wrap_angle(yaw)
        self.pid.reset()

    def step(self, yaw: float, dt: float, pitch: float = 0.0) -> Tuple[float, float]:
        if self.target_yaw is None:
            self.set_target(yaw)
        e = wrap_angle(self.target_yaw - yaw)
        w = self.pid.step(e, dt)
        # 大きなヨー誤差では減速、急斜面 (ピッチ大) でも減速
        v = self.v_nominal * max(0.3, 1.0 - abs(e) / math.radians(40))
        v *= max(0.5, 1.0 - abs(pitch) / math.radians(25))
        return v, w


def quat_to_euler(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    """四元数 → (roll, pitch, yaw)"""
    sinr = 2 * (w * x + y * z)
    cosr = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)
    siny = 2 * (w * z + x * y)
    cosy = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny, cosy)
    return roll, pitch, yaw
