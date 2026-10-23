#!/usr/bin/env bash
# デモ走行を録画する (WSL2 ホスト側で実行。ffmpeg と xwininfo が必要)
#
#   solar_mower_sim/scripts/record_demo.sh [出力.mp4] [コマンド JSON]
#   既定: videos/mow_lane.mp4, {"action":"mow_lane","length":11}
#
# 前提: シミュレーションが起動中 (./sim ros2 launch solar_mower_sim sim.launch.py)。
# GUI を録画用設定 (config/gui_record.config: 3D ビューのみ 1280x768) で開き直し、ワールドをリセットしてから
# Gazebo のウィンドウを ffmpeg (x11grab) で録画し、/mission_status が idle になったら止める。
# gz-sim 8 の VideoRecorder はサービスから操作できないので画面録画にしている。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM="$ROOT/sim"
OUT="${1:-$ROOT/videos/mow_lane.mp4}"
DEFAULT_CMD='{"action":"mow_lane","length":11}'
CMD="${2:-$DEFAULT_CMD}"
CFG=/ws/install/solar_mower_sim/share/solar_mower_sim/config/gui_record.config
mkdir -p "$(dirname "$OUT")"
export DISPLAY="${DISPLAY:-:0}"

echo "== GUI を録画用設定で開き直す"
"$SIM" pkill -f "gz sim -g" || true
sleep 2
"$SIM" gz sim -g --gui-config "$CFG" >/tmp/gz_gui_record.log 2>&1 &
WIN=""
for _ in $(seq 1 60); do
  sleep 1
  WIN=$(xwininfo -root -tree | awk '/"Gazebo Sim": \("gz-sim-gui"/ {print $1; exit}')
  [ -n "$WIN" ] && break
done
[ -n "$WIN" ] || { echo "Gazebo のウィンドウが見つからない"; exit 1; }
sleep 8   # シーンの読み込み待ち

echo "== ワールドをリセット"
"$SIM" gz service -s /world/solar_farm/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'reset: {all: true}' >/dev/null
sleep 3

echo "== 録画開始 ($OUT)"
ffmpeg -loglevel error -y -f x11grab -draw_mouse 0 -framerate 30 -window_id "$WIN" -i "$DISPLAY" \
  -c:v libx264 -preset veryfast -pix_fmt yuv420p -crf 20 "$OUT" &
FF=$!
sleep 2

echo "== コマンド送信: $CMD"
"$SIM" bash -c "ros2 topic echo /mission_status --field data | grep -m1 -x idle" &
WAIT=$!
sleep 2
"$SIM" ros2 topic pub --once /mission_cmd std_msgs/msg/String "{data: '$CMD'}" >/dev/null
wait "$WAIT"
sleep 2

echo "== 録画停止"
kill -TERM "$FF"; wait "$FF" || true   # バックグラウンドの ffmpeg は SIGINT を無視するので TERM (trailer は書かれる)
ffprobe -v error -show_entries format=duration -of default=nw=1 "$OUT"
echo "saved: $OUT"
