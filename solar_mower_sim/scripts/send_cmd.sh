#!/usr/bin/env bash
# LLM を介さず JSON コマンドを直接送る (デバッグ用)
#   scripts/send_cmd.sh '{"action":"follow_panel","side":"left","distance":0.9,"length":10}'
set -e
ros2 topic pub --once /mission_cmd std_msgs/msg/String "{data: '$1'}"
