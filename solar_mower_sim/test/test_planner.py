from solar_mower_sim.planner import lane_offsets, plan_mow_lane


def test_lane_offsets_cover_lane_without_gaps():
    offs = lane_offsets(3.8, 0.6, 0.6)
    assert offs[0] == 0.6 and abs(offs[-1] - 3.2) < 1e-9
    assert all(b - a <= 0.6 + 1e-9 for a, b in zip(offs, offs[1:]))


def test_plan_from_center_left_start():
    steps = plan_mow_lane(1.9, 1.9, length=11)
    # まず左 (北) の端へ移動
    assert steps[0] == {"action": "turn", "angle_deg": 90.0}
    assert steps[1]["action"] == "go_straight" and abs(steps[1]["distance"] - 1.3) < 1e-6
    assert steps[2] == {"action": "turn", "angle_deg": -90.0}
    passes = [s for s in steps if s["action"] == "follow_panel"]
    assert len(passes) == len(lane_offsets(3.8, 0.6, 0.6))
    # 往路は北の列が左、復路は北の列が右。中央より南のパスは南の列を基準にする
    assert passes[0]["side"] == "left" and passes[0]["distance"] == 0.6
    assert passes[1]["side"] == "right"
    assert passes[-1]["distance"] == 0.6
    assert all(p["distance"] <= 1.9 + 1e-9 for p in passes)
    # パスの間は同じ向きに 2 回 90°旋回 (= Uターン)。往路の後は右、復路の後は左
    turns = [s["angle_deg"] for s in steps[3:] if s["action"] == "turn"]
    assert turns[:4] == [-90.0, -90.0, 90.0, 90.0]


def test_plan_right_start_is_mirrored():
    left = plan_mow_lane(1.9, 1.9, length=11, start_side="left")
    right = plan_mow_lane(1.9, 1.9, length=11, start_side="right")
    flip = {"left": "right", "right": "left"}
    for a, b in zip(left, right):
        assert a["action"] == b["action"]
        if a["action"] == "turn":
            assert a["angle_deg"] == -b["angle_deg"]
        if a["action"] == "follow_panel":
            assert flip[a["side"]] == b["side"] and a["distance"] == b["distance"]
