import math
import numpy as np
from solar_mower_sim.controllers import (HeadingController, LineFit, LineFitter,
                                         PanelFollowController, quat_to_euler)


def _line_pts(y0, slope, n=60, noise=0.01):
    x = np.linspace(-2, 3, n)
    pts = np.stack([x, y0 + slope * x], 1)
    pts += np.random.default_rng(1).normal(0, noise, pts.shape)
    return np.vstack([pts, np.random.default_rng(2).uniform(-3, 3, (15, 2))])


def test_ransac_parallel_left():
    fit = LineFitter().fit(_line_pts(0.9, 0.0))
    assert abs(fit.distance - 0.9) < 0.03
    assert abs(fit.angle) < math.radians(2)
    assert fit.side == 1


def test_ransac_right_tilted():
    fit = LineFitter().fit(_line_pts(-0.9, 0.2))
    assert fit.side == -1
    assert abs(math.degrees(fit.angle) - 11.3) < 1.0


def test_follow_turns_toward_panel_when_far():
    c = PanelFollowController(target_dist=0.9, side=1)
    _, w, st = c.step(LineFit(0, 1, 1.3, 30, 30), 0.1)
    assert st == "ok" and w > 0
    c = PanelFollowController(target_dist=0.9, side=-1)
    _, w, _ = c.step(LineFit(0, -1, 1.3, 30, 30), 0.1)
    assert w < 0


def test_follow_stops_when_lost():
    c = PanelFollowController(lost_timeout=0.5)
    st = None
    for _ in range(10):
        _, _, st = c.step(None, 0.1)
    assert st == "stopped"


def test_heading_corrects_sign():
    h = HeadingController()
    h.set_target(0.0)
    _, w = h.step(math.radians(10), 0.1)
    assert w < 0


def test_quat():
    r, p, y = quat_to_euler(0, 0, math.sin(0.5), math.cos(0.5))
    assert abs(y - 1.0) < 1e-6
