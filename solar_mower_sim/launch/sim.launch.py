"""Gazebo Harmonic + ros_gz_bridge + mission_executor を起動する。

  ros2 launch solar_mower_sim sim.launch.py            # GUI あり
  ros2 launch solar_mower_sim sim.launch.py gui:=false # ヘッドレス
  ros2 launch solar_mower_sim sim.launch.py llm:=true  # llm_commander も同時起動 (別端末推奨)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("solar_mower_sim")
    ros_gz = get_package_share_directory("ros_gz_sim")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    llm = LaunchConfiguration("llm")

    headless = PythonExpression(["'' if '", gui, "' == 'true' else '-s '"])
    gz_args = ["-r ", headless, os.path.join(pkg, "worlds", ""), world]

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value="solar_farm.sdf"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("llm", default_value="false"),

        # モデル検索パス (model://tracked_mower 等)
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH",
                               os.path.join(pkg, "models") + ":" + os.path.join(pkg, "worlds")),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz, "launch", "gz_sim.launch.py")),
            launch_arguments={"gz_args": gz_args}.items(),
        ),

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
