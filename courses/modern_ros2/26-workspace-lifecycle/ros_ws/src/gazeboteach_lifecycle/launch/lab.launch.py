"""Start the installed lifecycle node and independent native sensor bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        LifecycleNode(
            package="gazeboteach_lifecycle", executable="sensor", name="sensor_lifecycle",
            namespace="", output="screen",
            parameters=[{"use_sim_time": ParameterValue(LaunchConfiguration("use_sim_time"), value_type=bool)}],
        ),
        Node(
            package="ros_gz_bridge", executable="parameter_bridge", name="m26_bridge", output="screen",
            arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                       "/gt/lifecycle/raw_imu@sensor_msgs/msg/Imu[gz.msgs.IMU"],
        ),
    ])
