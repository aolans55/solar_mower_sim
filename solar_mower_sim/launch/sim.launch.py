"""Gazebo Harmonic + ros_gz_bridge + mission_executor を起動する。

  ros2 launch solar_mower_sim sim.launch.py            # GUI あり
  ros2 launch solar_mower_sim sim.launch.py gui:=false # ヘッドレス
  ros2 launch solar_mower_sim sim.launch.py llm:=true  # llm_commander も同時起動 (別端末推奨)

Gazebo はサーバー (物理・センサー) と GUI を別プロセスで起動する。環境変数 GZ_SERVER_GALLIUM_DRIVER が
あればサーバーだけその Mesa ドライバで描画する (WSL2 では llvmpipe: d3d12 だと gpu_lidar の描画で
セグメンテーション違反が起きるため。compose.yaml で設定)。
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("solar_mower_sim")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    llm = LaunchConfiguration("llm")

    world_path = PathJoinSubstitution([pkg, "worlds", world])
    server_env = {}
    if os.environ.get("GZ_SERVER_GALLIUM_DRIVER"):
        server_env["GALLIUM_DRIVER"] = os.environ["GZ_SERVER_GALLIUM_DRIVER"]

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="solar_farm.sdf"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("llm", default_value="false"),

        # モデル検索パス (model://tracked_mower 等)
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(pkg, "models") + ":" + os.path.join(pkg, "worlds")),

        ExecuteProcess(cmd=["gz", "sim", "-s", "-r", world_path],
                       additional_env=server_env, output="screen", name="gz_server"),
        # GUI のカメラ初期位置は config/gui.config で指定 (列の間を見下ろす)
        ExecuteProcess(cmd=["gz", "sim", "-g", "--gui-config",
                            os.path.join(pkg, "config", "gui.config")],
                       output="screen", name="gz_gui", condition=IfCondition(gui)),

        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="bridge",
            parameters=[{"config_file": os.path.join(pkg, "config", "bridge.yaml"),
                         "use_sim_time": True}],
            output="screen",
        ),

        Node(
            package="solar_mower_sim",
            executable="mission_executor",
            name="mission_executor",
            parameters=[{"use_sim_time": True}],
            output="screen",
        ),

        Node(
            package="solar_mower_sim",
            executable="llm_commander",
            name="llm_commander",
            parameters=[{"interactive": False}],
            output="screen",
            condition=IfCondition(llm),
        ),
    ])
