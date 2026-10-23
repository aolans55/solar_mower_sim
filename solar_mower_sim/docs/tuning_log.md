# 調整ログ

調整したパラメータと理由。レポート (4・5 節) の材料。

## 2026-10-23 WSL2 + Docker での初回動作確認

### 地形テクスチャ (ワールド読み込み失敗)
- 症状: `Failed to load a world`。heightmap が Gazebo Classic の `file://media/materials/textures/*.png` を参照していた
- 対処: `scripts/gen_terrain_textures.py` で草・土・平坦法線を生成し `model://` で参照

### 壁追従: LiDAR 高さと横梁 (follow_panel が 0.25 m で "panel lost")
- 症状: 走り出すと横梁を見失い、支柱 (6〜11 点) しか見えなくなる
- 原因: ワールド座標で LiDAR 0.63 m、横梁 0.62〜0.72 m → 横梁下端の 1 cm 上しかなく、ロール 0.5〜2° で外れる
- 対処:
  | パラメータ | 変更前 | 変更後 |
  |---|---|---|
  | `tracked_mower` lidar_link z | 0.33 | 0.37 (横梁中心 0.67 m に一致) |
  | `gen_panel_row.py` 横梁上下幅 `BEAM_H` | 0.10 | 0.15 |
- 結果 (直進 8 m で計測): 横梁検出 83/83 (変更前は約 7 割が未検出)
- 結果 (`follow_panel left 0.9 length 10`): 推定距離 平均 0.898 m (0.874〜0.919)、真値 0.901 m (0.877〜0.913)、角度誤差 最大 1.0°、見失い 0/84、10 m 到達で正常終了

### 制御タイマーをシミュレーション時刻に
- 症状: WSL2 では RTF ≈ 0.33 (headless) / 0.25 (GUI)。`time.monotonic()` だと PID の dt が実際の 3〜4 倍になり、I 項が過大・D 項が過小になる
- 対処: `mission_executor` の dt / duration を ROS 時刻 (`use_sim_time`) で計算
- 結果: cmd_vel 20 Hz (シミュレーション時刻)、速度 0.494 m/s で指令通り
