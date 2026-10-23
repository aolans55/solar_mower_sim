# ROS 2 Jazzy + Gazebo Harmonic のシミュレーション環境 (WSL2 + Docker Desktop 向け)
#   ホストの WSL には何もインストールしない。ソースは compose でマウントする。
FROM osrf/ros:jazzy-desktop

RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz \
      python3-numpy python3-requests python3-pil python3-pytest \
      mesa-utils \
      xterm xdotool fonts-noto-cjk \
    && rm -rf /var/lib/apt/lists/*

# ベースイメージの ubuntu ユーザー (uid 1000 = WSL 側のユーザーと同じ) で動かし、
# マウントしたソースに root 所有のファイルを作らないようにする
RUN mkdir -p /ws/src /ws/build /ws/install /ws/log && chown -R 1000:1000 /ws
COPY entrypoint.sh /entrypoint.sh
USER ubuntu
WORKDIR /ws

ENTRYPOINT ["/entrypoint.sh"]
CMD ["sleep", "infinity"]
