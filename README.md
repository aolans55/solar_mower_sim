# solar_mower_sim — ソーラー発電所向け 履帯式草刈りロボットの LLM 制御シミュレーション

Physical AI 応用1講座 最終課題「AI×ロボティクスによる創造的タスクの実現」の提出用リポジトリ。

ソーラー発電所の草刈り作業を想定し、仮想環境 (ROS 2 Jazzy + Gazebo Harmonic) 内で
履帯（クローラ）ロボットが次の 3 つを行う。

| # | 機能 | 使う技術 |
|---|------|---------|
| 1 | **自然言語の指示で動く** | ローカル LLM (OpenAI 互換 API: LM Studio / Ollama) が日本語指示 → JSON コマンドに変換。制御ループには LLM を入れない安全設計 |
| 2 | **パネル列に平行に走る** | 2D LiDAR 点群から架台の横梁を RANSAC で直線抽出 → 距離・角度の 2 自由度 PID で壁追従 |
| 3 | **凸凹地面を乗り越えつつ直進** | heightmap 不整地 + 履帯物理 (TrackedVehicle)。IMU ヨー角の方位保持 PID で履帯スリップによる進路ずれを補正 |

```
 オペレータ ──日本語──▶ llm_commander (LLM) ──JSON──▶ mission_executor ──/cmd_vel──▶ Gazebo
                                                             ▲  ▲  ▲
                                                 /scan  /imu  /odom (ros_gz_bridge)
```

## 1. 環境構築 (DGX Spark / Ubuntu 24.04 arm64)

```bash
# ROS 2 Jazzy
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install -y ros-jazzy-desktop ros-dev-tools

# Gazebo Harmonic + ROS-Gazebo ブリッジ
sudo apt install -y ros-jazzy-ros-gz

# Python 依存
sudo apt install -y python3-numpy python3-requests python3-pil python3-pytest

# Ollama (ローカル LLM, DGX Spark で LLM も動かす場合)
curl -fsSL https://ollama.com/install.sh | sh
ollama pull qwen2.5:7b        # 日本語が得意な 7B。余裕があれば qwen2.5:14b / 32b
```

`scripts/setup_dgx_spark.sh` に上記をまとめてある。WSL2 では `scripts/setup_wsl2.sh`
(LLM は Windows 側の LM Studio を使うので WSL には入れない)。

### LLM サーバーの選択 (OpenAI 互換 API)

`llm_commander` は OpenAI 互換の `/v1/chat/completions` を呼ぶ。環境変数 (または同名の ROS パラメータ
`base_url` / `model` / `api_key` / `reasoning_effort`) で接続先を切り替える。

| 変数 | 既定 | 説明 |
|------|------|------|
| `LLM_BASE_URL` | `http://localhost:1234/v1` (LM Studio) | 未設定で `OLLAMA_HOST` があれば `$OLLAMA_HOST/v1` |
| `LLM_MODEL` | 空 = サーバーの先頭のチャットモデル | 例: `qwen3.5-9b-mtp`, `qwen2.5:7b` |
| `LLM_API_KEY` | 空 | ローカルサーバーなら不要 |
| `LLM_REASONING_EFFORT` | `none` | 思考モデルの推論を止めて高速化。空にすると送らない (400 なら自動で外して再送) |

```bash
# Windows の LM Studio (Developer タブで Start Server。WSL2 は mirrored ネットワークで localhost が届く)
export LLM_BASE_URL=http://localhost:1234/v1 LLM_MODEL=qwen3.5-9b-mtp
# 外部 PC (DGX Spark) の Ollama。DGX 側は OLLAMA_HOST=0.0.0.0 で起動しておく
export LLM_BASE_URL=http://<dgx-spark>:11434/v1 LLM_MODEL=qwen2.5:7b
```

### WSL2 (Windows) では Docker で動かす

ホストの WSL を汚さないよう、ROS 2 + Gazebo はコンテナに入れる (Docker Desktop の WSL 連携を有効にしておく)。
GUI / GPU は WSLg をコンテナに渡し、LLM は Windows 側の LM Studio (`host.docker.internal:1234`) を使う。

```bash
docker compose up -d --build                     # 初回はイメージ作成で数十分
./sim colcon build --symlink-install             # ビルド (結果は Docker volume に保存)
./sim ros2 launch solar_mower_sim sim.launch.py
./sim ros2 run solar_mower_sim llm_commander     # 別端末
./sim                                            # コンテナ内のシェル (ros2 topic echo 等)
```

`./sim <cmd>` はコンテナ内で ROS 環境を読み込んでからコマンドを実行する
(`docker compose exec` を直接使うと ROS 環境が読み込まれない)。

GPU は Mesa の d3d12 ドライバ経由 (`GALLIUM_DRIVER=d3d12`)。描画がおかしい場合は `GALLIUM_DRIVER=llvmpipe docker compose up -d` でソフトウェア描画に切り替える。
WSL に直接入れる場合は `scripts/setup_wsl2.sh` (apt でシステムに入る)。

## 2. ビルド

```bash
mkdir -p ~/pai_ws/src && cd ~/pai_ws/src
git clone <このリポジトリ> solar_mower_sim
cd ~/pai_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. 実行

### 端末 1: シミュレータ + 制御ノード
```bash
source ~/pai_ws/install/setup.bash
ros2 launch solar_mower_sim sim.launch.py
```
Gazebo が開き、パネル列 2 本と不整地、その南側に橙色の履帯ロボットが現れる。
GUI 左下の ▶ で再生が始まっていなければ押す (`-r` で自動再生指定済み)。

### 端末 2: 自然言語コマンド
```bash
source ~/pai_ws/install/setup.bash
ros2 run solar_mower_sim llm_commander
> パネルに沿って10メートル進んで
> 右に90度曲がって、北に向かって8メートル真っ直ぐ進んで
> 止まれ
```
変換結果は `/llm_log`、実行状況は `/mission_status` で確認できる:
```bash
ros2 topic echo /mission_status
```

### LLM を使わず JSON を直接送る (デバッグ)
```bash
scripts/send_cmd.sh '{"action":"follow_panel","side":"left","distance":0.9,"length":10}'
scripts/send_cmd.sh '{"action":"go_straight","distance":10,"heading":"east"}'
```

### LLM の変換精度だけ確認 (ROS 不要)
```bash
python3 scripts/test_llm.py              # LLM_BASE_URL / LLM_MODEL の設定で接続
python3 scripts/test_llm.py qwen3.5-9b-mtp   # モデルだけ指定
```

### 単体テスト
```bash
cd solar_mower_sim && python3 -m pytest test
```

## 4. JSON コマンド仕様

| action | フィールド | 説明 |
|--------|-----------|------|
| `follow_panel` | `side` (left/right), `distance` [m], `speed` [m/s], `length` [m] | 架台に沿って走行。`length` 省略で見失うまで |
| `go_straight` | `distance` [m], `speed`, `heading` (north/south/east/west/deg) | IMU 方位保持で直進。`heading` 省略で現在方位 |
| `turn` | `angle_deg` (左回り正) | その場旋回 |
| `stop` | — | 停止・キュー消去 |
| `sequence` | `steps[]` | 順次実行 |

`llm_commander` 側でスキーマ検証と数値クランプ (速度 ≤ 1.0 m/s 等) を行うので、LLM の誤出力は駆動系に届かない。

## 5. ディレクトリ構成

```
solar_mower_sim/
├── solar_mower_sim/
│   ├── controllers.py        # RANSAC 直線抽出・壁追従 PID・方位保持 PID (ROS 非依存)
│   ├── mission_executor.py   # JSON コマンドの状態機械 → /cmd_vel
│   ├── llm_client.py         # OpenAI 互換 API で自然言語 → JSON、スキーマ検証 (ROS 非依存)
│   └── llm_commander.py      # 上記を使う ROS ノード
├── models/
│   ├── tracked_mower/        # 履帯ロボット (TrackedVehicle + TrackController, LiDAR, IMU)
│   ├── panel_row/            # パネル列 (scripts/gen_panel_row.py で生成)
│   └── solar_farm_terrain/   # 不整地 heightmap (scripts/gen_heightmap.py) とテクスチャ (scripts/gen_terrain_textures.py)
├── worlds/solar_farm.sdf
├── launch/sim.launch.py
├── config/bridge.yaml        # ros_gz_bridge トピック対応
├── prompts/system_prompt.txt # LLM 用システムプロンプト (few-shot)
├── scripts/                  # 生成・デバッグ用スクリプト
└── test/                     # pytest
```

## 6. 調整のヒント (実機で最初に試すこと)

- **履帯が動かない / 滑る**: `models/tracked_mower/model.sdf` の `<mu>`, `<kp>`, `<tracks_separation>`、ワールドの `max_step_size` (0.001 → 0.0005) を調整。TrackController は DART 物理エンジン前提。
- **壁追従が振動する**: `PanelFollowController` の `kp_dist` を下げ `kd_dist` を上げる。`inlier_thresh` (6 cm) を広げると不整地での LiDAR 傾きに強くなる。
- **直進で蛇行する**: `HeadingController` の `kp` を下げる。急斜面では `pitch` で自動減速。
- **LiDAR が架台を捉えない**: LiDAR 高さ (0.58 m) と架台横梁高さ (0.55 m, `gen_panel_row.py` の `BEAM_Z`) の関係を確認。
- **履帯プラグインがどうしても安定しない場合の保険**: `DiffDrive` プラグイン + 4 輪に差し替え (履帯の見た目は維持)。制御ノード側は無変更で動く。

## 7. 動画構成案 (3 分)

1. (0:00–0:30) 課題の説明テロップ → 端末で日本語指示を入力し JSON に変換される様子
2. (0:30–1:30) 俯瞰カメラ: パネル列に沿って走行。LiDAR 点群表示を ON (Gazebo GUI の「Visualize Lidar」プラグイン)
3. (1:30–2:30) 側面カメラ: 列間の不整地を直進。`ros2 topic echo /imu` のヨー角または rqt_plot を画面端に重ねる
4. (2:30–3:00) 「右に曲がって次の列へ」の sequence 実行 → 停止。まとめテロップ

録画は `gz sim` の録画プラグインか OBS。倍速にした場合は動画内に明記。

## 8. 提出物チェックリスト (課題要項より)

- [ ] 動画 1〜3 分 (YouTube 限定公開以上、タイトル `PAI最終課題_<omnicampusアカウント名>`)
- [ ] GitHub リポジトリ (public)
- [ ] レポート PDF `説明資料_PAI最終課題_<アカウント名>.pdf` (`docs/report_template.md` を元に作成)
- [ ] 個人情報 (氏名・所属・メール) を成果物に含めない
- [ ] 締切 2026-11-02 (月) 10:00
