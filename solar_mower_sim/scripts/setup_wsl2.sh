#!/usr/bin/env bash
# WSL2 (Ubuntu 24.04 x86_64) 向け一括セットアップ
#   - ROS 2 Jazzy + Gazebo Harmonic を apt で導入し、~/pai_ws でビルドする
#   - LLM は Windows 側の LM Studio (OpenAI 互換 API, localhost:1234) を使う前提 (WSL には入れない)
#   - conda (miniconda) の python が混ざると colcon build が壊れるので PATH から外す
set -euo pipefail

# conda を PATH から除外 (ROS 2 Jazzy はシステムの python3.12 前提)
PATH=$(echo "$PATH" | tr ':' '\n' | grep -v -e conda -e miniconda | paste -sd:)
export PATH
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONPATH || true
echo "python3 = $(command -v python3) ($(python3 --version))"

echo "== ROS 2 Jazzy + Gazebo Harmonic =="
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  python3-numpy python3-requests python3-pil python3-pytest python3-colcon-common-extensions \
  mesa-utils

echo "== workspace =="
mkdir -p ~/pai_ws/src
if [ ! -e ~/pai_ws/src/solar_mower_sim ]; then
  ln -s "$(cd "$(dirname "$0")/.." && pwd)" ~/pai_ws/src/solar_mower_sim
fi
cd ~/pai_ws
set +u; source /opt/ros/jazzy/setup.bash; set -u
colcon build --symlink-install

echo
echo "== checks =="
glxinfo -B 2>/dev/null | grep -E "OpenGL renderer|OpenGL version" || echo "glxinfo failed"
curl -s -m 3 "${LLM_BASE_URL:-http://localhost:1234/v1}/models" >/dev/null && echo "LLM server OK: ${LLM_BASE_URL:-http://localhost:1234/v1}" \
  || echo "LLM サーバーに届かない: Windows の LM Studio で Developer タブ → Start Server"
echo
echo "done. next (conda deactivate してから):"
echo "  source ~/pai_ws/install/setup.bash"
echo "  ros2 launch solar_mower_sim sim.launch.py"
