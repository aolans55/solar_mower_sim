#!/usr/bin/env bash
# 「日本語で指示 → LLM が JSON を生成 → ロボットが実行」を 1 画面で録画する
# (WSL2 ホスト側で実行。ffmpeg と xwininfo が必要。シミュレーション起動中に)
#
#   solar_mower_sim/scripts/record_llm_demo.sh [出力.mp4] [日本語の指示]
#   既定: videos/llm_demo.mp4, 「このレーンを端から端まで往復して草刈りして」
#
# 画面構成 (1920x1080):
#   左上: llm_commander の端末 (xdotool で指示を入力)   右: Gazebo (3D ビュー)
#   左下: /llm_log と /mission_status (show_llm_log.py)
# 端末・Gazebo はコンテナ内で開き、各ウィンドウを ffmpeg (x11grab) で同時に撮って並べる。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SIM="$ROOT/sim"
OUT="${1:-$ROOT/videos/llm_demo.mp4}"
TEXT="${2:-このレーンを端から端まで往復して草刈りして}"
MODEL="${LLM_MODEL:-qwen3.5-9b-mtp}"
FONT="${LABEL_FONT:-/mnt/c/Windows/Fonts/BIZ-UDGothicB.ttc}"
SHARE=/ws/install/solar_mower_sim/share/solar_mower_sim
mkdir -p "$(dirname "$OUT")"
export DISPLAY="${DISPLAY:-:0}"

winid() {  # ウィンドウ名 → X ウィンドウ ID (現れるまで待つ)
  local id=""
  for _ in $(seq 1 60); do
    id=$(xwininfo -root -tree | grep -F "\"$1\": (" | awk '{print $1; exit}')
    [ -n "$id" ] && { echo "$id"; return; }
    sleep 1
  done
  echo "window '$1' not found" >&2; exit 1
}

echo "== LLM を読み込んでおく ($MODEL)"
"$SIM" curl -s -m 300 "${LLM_BASE_URL:-http://host.docker.internal:1234/v1}/chat/completions" \
  -H 'Content-Type: application/json' \
  -d "{\"model\":\"$MODEL\",\"max_tokens\":1,\"reasoning_effort\":\"none\",\"messages\":[{\"role\":\"user\",\"content\":\"hi\"}]}" >/dev/null

echo "== ウィンドウを開く"
"$SIM" pkill -f "gz sim -g" || true
"$SIM" pkill xterm || true
sleep 2
# Gazebo: 録画用設定をウィンドウ 770x850 にして開き、画面右に置く
# (WSLg の画面は 1440x900。はみ出したウィンドウは x11grab で撮れないので全部を画面内に収める)
"$SIM" bash -c "sed 's#<width>1280</width>#<width>770</width>#; s#<height>768</height>#<height>850</height>#' \
  $SHARE/config/gui_record.config > /tmp/gui_llm_demo.config"
"$SIM" gz sim -g --gui-config /tmp/gui_llm_demo.config >/tmp/gz_gui_llm_demo.log 2>&1 &
XT=(xterm -fa "DejaVu Sans Mono" -fd "Noto Sans Mono CJK JP" -fs 12 -bg "#1e1e1e" -fg "#e0e0e0")   # 1 文字 約 10x19 px
ROS_ENV="source /opt/ros/jazzy/setup.bash; source /ws/install/setup.bash; export RCUTILS_CONSOLE_OUTPUT_FORMAT='[{severity}] {message}'"
"$SIM" "${XT[@]}" -T operator -geometry 64x20+0+0 -e bash -c \
  "$ROS_ENV; LLM_MODEL=$MODEL ros2 run solar_mower_sim llm_commander; sleep 3600" &
"$SIM" "${XT[@]}" -T llm_log -geometry 64x20+0+440 -e bash -c \
  "$ROS_ENV; python3 /ws/src/solar_mower_sim/scripts/show_llm_log.py; sleep 3600" &
GZ=$(winid "Gazebo Sim"); OP=$(winid operator); LG=$(winid llm_log)
"$SIM" xdotool search --name "^Gazebo Sim$" windowmove 655 0 >/dev/null || true
sleep 10   # シーン・ノードの起動待ち

echo "== ワールドをリセット"
"$SIM" gz service -s /world/solar_farm/control --reqtype gz.msgs.WorldControl \
  --reptype gz.msgs.Boolean --timeout 5000 --req 'reset: {all: true}' >/dev/null
sleep 3

echo "== 録画開始 ($OUT)"
lab() { echo "pad=768:540:0:48:color=0x2b2b2b,drawtext=fontfile=$FONT:text='$1':fontcolor=white:fontsize=26:x=16:y=11"; }
TERM_FIT="scale=768:492:force_original_aspect_ratio=decrease,pad=768:492:(ow-iw)/2:(oh-ih)/2:color=0x1e1e1e"
ffmpeg -loglevel error -y \
  -f x11grab -draw_mouse 0 -framerate 30 -window_id "$OP" -i "$DISPLAY" \
  -f x11grab -draw_mouse 0 -framerate 30 -window_id "$LG" -i "$DISPLAY" \
  -f x11grab -draw_mouse 0 -framerate 30 -window_id "$GZ" -i "$DISPLAY" \
  -filter_complex "\
[0:v]$TERM_FIT,$(lab '① 日本語で指示  (llm_commander)')[op];\
[1:v]$TERM_FIT,$(lab '② LLM が生成したコマンド  (/llm_log)')[lg];\
[op][lg]vstack[left];\
[2:v]scale=1152:1080:force_original_aspect_ratio=decrease,pad=1152:1080:(ow-iw)/2:(oh-ih)/2:color=black[gz];\
[left][gz]hstack,format=yuv420p[v]" \
  -map "[v]" -c:v libx264 -preset veryfast -crf 20 "$OUT" &
FF=$!
sleep 3

echo "== 指示を入力: $TEXT"
# grep が idle を見つけたら echo も止める (パイプだと echo が次のメッセージまで終わらない)
"$SIM" bash -c "grep -m1 -x idle < <(ros2 topic echo /mission_status std_msgs/msg/String --field data); \
  pkill -f 'topic echo /mission_status' || true" &
WAIT=$!
sleep 1
# xdotool search ... type と連結すると入力先に届かないので、ID を取ってから --window で送る
"$SIM" bash -c 'W=$(xdotool search --name "^operator$" | head -1); xdotool type --window $W --delay 150 "$1"; sleep 1; xdotool key --window $W Return' _ "$TEXT"
wait "$WAIT"
sleep 3

echo "== 録画停止"
kill -TERM "$FF"; wait "$FF" || true   # バックグラウンドの ffmpeg は SIGINT を無視するので TERM
ffprobe -v error -show_entries format=duration -of default=nw=1 "$OUT"
echo "saved: $OUT"
