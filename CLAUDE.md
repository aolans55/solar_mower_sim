# CLAUDE.md — 引継ぎ資料（WSL2 上の Claude Code 向け）

このファイルはリポジトリ直下に置く Claude Code 用のプロジェクト指示書を兼ねる。
Claude Code は起動時にこれを自動で読む。人間が読んでも分かるように書いてある。

## 0. このプロジェクトは何か

- Physical AI 応用1講座（東大松尾研）の最終課題「AI×ロボティクスによる創造的タスクの実現」
- 題材: **ソーラー発電所向け 履帯式草刈りロボット**の仮想環境デモ。ユーザーは本業で同用途の草刈り機を開発予定で、その制御系のプロトタイプも兼ねる
- 必須デモ 3 機能:
  1. 自然言語の指示で動く（ローカル LLM: OpenAI 互換 API。WSL2 では LM Studio、DGX Spark では Ollama）
  2. ソーラーパネル列（架台）に平行に走る（LiDAR + RANSAC + PID 壁追従）
  3. 凸凹地面を乗り越えながら直進する（heightmap + 履帯物理 + IMU 方位保持 PID）
- **締切: 2026-11-02 (月) 10:00**。提出物は (a) 動画 1〜3 分 YouTube 限定公開以上、(b) GitHub public リポジトリ、(c) レポート PDF。課題要項の要点は README 8 節と `docs/report_template.md` を参照
- **成果物に氏名・所属・メールを一切含めない**（要項の禁止事項）。コミット作者名も個人名を避ける

## 1. 現在の状態（2026-10-23 時点）

| 項目 | 状態 |
|---|---|
| ロボット・ワールド SDF | WSL2 + Docker の Gazebo で起動確認済み（2026-10-23）。地形テクスチャが Classic のパスで読めずワールドが落ちていたのを修正（`scripts/gen_terrain_textures.py` で生成）。`go_straight 3m` で実位置も 3.04 m 前進、RTF ≈ 1.0 |
| 制御ロジック `controllers.py` | 作成済み。pytest 6 件 PASS（ROS 非依存、コンテナ内でも PASS） |
| `mission_executor.py` / `llm_commander.py` | executor は go_straight / follow_panel（10 m、距離 0.898±0.025 m、角度 ±1°）を確認。タイマーをシミュレーション時刻に変更。turn / llm_commander は未確認 |
| launch / bridge.yaml / setup.py | colcon build 成功。/clock /scan /imu /odom がブリッジされることを確認 |
| LLM プロンプト | 作成済み。LM Studio (`qwen3.5-9b-mtp`) で `test_llm.py` 8/8 OK（2026-10-23）。Ollama では未検証 |
| 環境構築 | WSL2 は Docker イメージ `solar-mower-sim:jazzy` 構築済み（`./sim` でコンテナ内実行）。DGX Spark は `setup_dgx_spark.sh` |

壁追従まで確認済み。次は 4 節の 5（不整地直進）から。WSL2 では RTF ≈ 0.25〜0.33（README 6 節）。

## 2. 開発環境の方針（決定事項）

- ユーザーの機材: DGX Spark（Ubuntu 24.04 arm64, 128GB）、Jetson Orin Nano、Windows 11 PC（Lunar Lake / Intel Arc 140V 内蔵 GPU、dGPU なし）
- **このセッションは Windows PC の WSL2 (Ubuntu 24.04)**。日常開発・動作確認はここで行う。履帯物理の最終調整とフル画質録画は DGX Spark で行う（同じリポジトリを git で共有）
- ROS 2 **Jazzy** + Gazebo **Harmonic**（`ros-jazzy-ros-gz`）。Humble/Fortress には変えない
- LLM は **OpenAI 互換 API**（`/v1/chat/completions`）のローカルサーバー（API キー不要、課題要項でも API 提供なし）。WSL2 では **Windows 側の LM Studio**（`http://localhost:1234/v1`、既定）、外部 PC（DGX Spark）では **Ollama**（`http://<host>:11434/v1`）。環境変数 `LLM_BASE_URL` / `LLM_MODEL` / `LLM_API_KEY` / `LLM_REASONING_EFFORT` で切り替える（README 1 節）
- 思考モデル（qwen3.5 等）は `reasoning_effort: none` を送らないと 1 回 40 秒以上かかる。既定で送っている
- ロボットは **履帯必須**（ユーザー要件）。Gazebo の `TrackedVehicle` + `TrackController` プラグインを使用。どうしても安定しない場合のみ `DiffDrive` + 4 輪に退避（制御ノードは無変更で動く設計）

### WSL2 固有の注意
- Intel 内蔵 GPU は Mesa d3d12 経由。`gpu_lidar` センサが動かなければ `models/tracked_mower/model.sdf` の `type="gpu_lidar"` を `type="lidar"`（CPU 版）に変更する。**launch 引数 `lidar:=cpu` で切り替えられるようにするのが TODO**
- ogre2 が起動時に落ちる場合: `export LIBGL_ALWAYS_SOFTWARE=1`
- `C:\Users\<user>\.wslconfig` で `memory=` を物理メモリの 2/3 程度に上げる（Gazebo 用。LLM は Windows 側の LM Studio で動かすので WSL のメモリは使わない）
- Windows 側の LM Studio には `.wslconfig` の `networkingMode=mirrored`（設定済み）で `localhost:1234` から届く。届かなければ LM Studio の Developer タブでサーバーが起動しているか確認
- conda (miniconda) の base が有効だと `python3` が 3.13 になり ROS 2 Jazzy (3.12) と衝突する。ROS を使う端末では `conda deactivate`
- 環境構築は Docker（`compose.yaml` + `docker/`、Docker Desktop の WSL 連携が必要）。WSLg の X11/Wayland と `/dev/dxg` をコンテナに渡し、`GALLIUM_DRIVER=d3d12` で GPU 描画する（指定しないと llvmpipe）。コンテナ内のコマンドは必ず `./sim <cmd>` で実行（`docker compose exec` 直だと ROS 環境が読まれない）。WSL への直接インストール（`scripts/setup_wsl2.sh`）は使わない方針
- Docker Desktop は WSL2 と同じ VM で動くので、`.wslconfig` の `memory=` がコンテナにも効く
- git の作者名はリポジトリローカルで `author <author@example.com>`

## 3. アーキテクチャ（触る前に把握すること）

```
端末入力/ /voice_text ─▶ llm_commander ──/mission_cmd (JSON)──▶ mission_executor ──/cmd_vel──▶ Gazebo TrackedVehicle
                         (LLM API, スキーマ検証,                 (状態機械 20Hz)        ▲ ▲ ▲
                          数値クランプ)                                        /scan /imu /odom ← ros_gz_bridge
```

- **LLM は制御ループに入れない**。JSON の生成だけを担当し、`llm_commander.validate()` でスキーマ検証と数値クランプ（速度 ≤1.0 m/s 等）を通す。この設計はレポートの工夫点①なので崩さない
- JSON アクション: `follow_panel / go_straight / turn / stop / sequence`。仕様は README 4 節
- `controllers.py` は ROS 非依存。ロジック変更時は `python3 -m pytest solar_mower_sim/test` を必ず通す
- 座標系: ロボット +x 前 / +y 左、ワールド +x 東 / +y 北。パネル列は x 方向に 12 m、`y=-8` と `y=+8`。架台前面の横梁（LiDAR の基準線）は列から南へ 0.47 m、高さ 0.55 m。ロボット初期位置 `(-7, -9.4)` 前方 +x、列1が左側
- LiDAR 高さは世界座標で約 0.67 m（lidar_link z 0.37 + 車体 0.30）。横梁は列 z 0.12 + `BEAM_Z` 0.55 = 中心 0.67 m、上下幅 `BEAM_H` 0.15 m（0.595〜0.745 m）。変える時は両方を一緒に動かす（2026-10-23 に合わせ直した。以前は LiDAR が横梁下端の 1 cm 上で、傾くと外れていた）
- パネル列と地形は生成物: `scripts/gen_panel_row.py`, `scripts/gen_heightmap.py`。SDF/PNG を手で直さず生成スクリプトを直す

## 4. 次にやること（優先順）

1. ~~**環境構築**~~（済）: WSL2 は **Docker**（ユーザー決定: ホストを汚さない）。`./sim colcon build --symlink-install`（README 1 節）。DGX Spark は `setup_dgx_spark.sh`。失敗したら README 1 節の手順を 1 行ずつ
2. **ビルド**: `cd ~/pai_ws && colcon build --symlink-install && source install/setup.bash`。setup.py の `data_files` でモデル・ワールド・プロンプトが `share/` に入るか確認
3. **Gazebo 起動**: `ros2 launch solar_mower_sim sim.launch.py`。確認項目:
   - ワールドが読めるか（heightmap の PNG 読み込み、`model://` 解決は `GZ_SIM_RESOURCE_PATH` を launch で設定済み）
   - 履帯が接地して沈まない／跳ねないか。沈むなら `<kp>`/`<min_depth>`、跳ねるなら `max_step_size` を 0.0005 に
   - `ros2 topic hz /scan /imu /odom` が出るか（bridge.yaml の型名を確認）
   - `scripts/send_cmd.sh '{"action":"go_straight","distance":3}'` で前進するか。動かなければ `gz topic -e -t /cmd_vel` で Gazebo 側に届いているか切り分け
4. **壁追従**: `scripts/send_cmd.sh '{"action":"follow_panel","side":"left","distance":0.9,"length":10}'`。振動するなら `PanelFollowController` の `kp_dist` ↓ `kd_dist` ↑。横梁を拾えていなければ LiDAR 高さ
5. **不整地直進**: ロボットを列間 `(-12, 0)` に置き `go_straight heading east distance 24`。マーカーポール（赤→緑）の間を走る。方位保持 ON/OFF での横ずれ差をレポート用に記録
6. **LLM**: `python3 solar_mower_sim/scripts/test_llm.py` で 8 例の変換を確認（LM Studio は確認済み。DGX Spark の Ollama は `LLM_BASE_URL=http://<host>:11434/v1 LLM_MODEL=qwen2.5:7b` で）。誤変換はプロンプトに例を追加して直す
7. **録画**: README 7 節の動画構成案。倍速にしたら動画内に明記
8. **レポート**: `docs/report_template.md` の数値を埋めて PDF 化。ファイル名 `説明資料_PAI最終課題_<omnicampusアカウント名>.pdf`

## 5. 作業ルール

- コミットは小さく。各コミットで pytest を通す
- 動かない箇所は「直す」か「README 6 節の調整ヒントに追記する」。両方しないまま次へ進まない
- 調整したパラメータ値と理由はレポートの材料になるので `docs/tuning_log.md`（新規作成してよい）に日付付きで残す
- 個人情報を書かない。`git config user.name` は `pai` 等の匿名にする
- 課題サンプルの丸写しは修了判定対象外。このリポジトリは独自設計だが、参考にしたサンプルはレポート 6 節に明記する

## 6. ファイル早見表

```
README.md                         使い方・調整ヒント・動画構成案・提出チェックリスト
CLAUDE.md                         本ファイル
solar_mower_sim/
  solar_mower_sim/controllers.py    RANSAC / 壁追従 PID / 方位保持 PID（ROS 非依存）
  solar_mower_sim/mission_executor.py  JSON 状態機械 → /cmd_vel
  solar_mower_sim/llm_client.py     OpenAI 互換 API 呼び出し + validate()（ROS 非依存）
  solar_mower_sim/llm_commander.py  上記を使う ROS ノード
  models/tracked_mower/model.sdf    履帯ロボット（プラグイン・LiDAR・IMU）
  models/panel_row/                 生成物 ← scripts/gen_panel_row.py
  models/solar_farm_terrain/        生成物 ← scripts/gen_heightmap.py
  worlds/solar_farm.sdf             ワールド（列 2 本・マーカー・ロボット初期位置）
  launch/sim.launch.py              gz_sim + bridge + executor (+ llm:=true)
  config/bridge.yaml                ros_gz_bridge
  prompts/system_prompt.txt         LLM システムプロンプト（few-shot）
  scripts/setup_dgx_spark.sh        環境構築一括
  scripts/send_cmd.sh               JSON 直送（LLM 抜きデバッグ）
  scripts/test_llm.py               LLM 変換テスト（ROS 不要）
  scripts/diag_panel_fit.py         LiDAR 横梁検出の診断（シミュレーション起動中に）
  scripts/gen_terrain_textures.py   地形テクスチャ生成
  docs/tuning_log.md                調整ログ（レポート材料）
  test/test_controllers.py          pytest
  docs/report_template.md           レポート雛形
```
