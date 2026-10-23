#!/usr/bin/env bash
# ROS 2 とワークスペース (ビルド済みなら) を読み込んでからコマンドを実行する
set -e
source /opt/ros/jazzy/setup.bash
if [ -f /ws/install/setup.bash ]; then
  source /ws/install/setup.bash
fi
exec "$@"
