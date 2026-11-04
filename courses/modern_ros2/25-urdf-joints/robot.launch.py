"""Launch robot_state_publisher and the three directed bridges for this URDF."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    folder = Path(__file__).resolve().parent
    description = (folder / "robot.urdf").read_text(encoding="utf-8")
    bridge = "bridge.fortress.yaml" if os.environ.get("GAZEBOTEACH_PROFILE") == "fortress-humble" else "bridge.yaml"
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="m25_state_publisher", output="screen",
             parameters=[{
                 "robot_description": ParameterValue(description, value_type=str),
                 "use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool),
                 "publish_frequency": 60.,
             }]),
        Node(package="ros_gz_bridge", executable="parameter_bridge", name="m25_bridge",
             output="screen", parameters=[{"config_file": str(folder / bridge)}]),
    ])
