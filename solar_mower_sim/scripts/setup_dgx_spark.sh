#!/usr/bin/env bash
# DGX Spark (Ubuntu 24.04 arm64) 向け一括セットアップ
set -euo pipefail

echo "== ROS 2 Jazzy =="
sudo apt update && sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz \
  python3-numpy python3-requests python3-pil python3-pytest python3-colcon-common-extensions

echo "== Ollama =="
if ! command -v ollama >/dev/null; then
  curl -fsSL https://ollama.com/install.sh | sh
fi
ollama pull "${OLLAMA_MODEL:-qwen2.5:7b}"

echo "== workspace =="
mkdir -p ~/pai_ws/src
if [ ! -e ~/pai_ws/src/solar_mower_sim ]; then
  ln -s "$(cd "$(dirname "$0")/.." && pwd)" ~/pai_ws/src/solar_mower_sim
fi
cd ~/pai_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
echo
echo "done. next:"
echo "  source ~/pai_ws/install/setup.bash"
echo "  ros2 launch solar_mower_sim sim.launch.py"
