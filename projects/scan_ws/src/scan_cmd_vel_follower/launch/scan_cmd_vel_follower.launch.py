from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("scan_cmd_vel_follower"))
    default_config = package_share / "config" / "follower.yaml"

    path_topic = LaunchConfiguration("path_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    cmd_topic = LaunchConfiguration("cmd_topic")
    planning_frame = LaunchConfiguration("planning_frame")
    max_linear_x = LaunchConfiguration("max_linear_x")
    max_angular_z = LaunchConfiguration("max_angular_z")
    lookahead_distance = LaunchConfiguration("lookahead_distance")

    return LaunchDescription(
        [
            DeclareLaunchArgument("path_topic", default_value="/visual_local_trajectory"),
            DeclareLaunchArgument("odom_topic", default_value="/Odometry"),
            DeclareLaunchArgument("cmd_topic", default_value="/cmd_vel"),
            DeclareLaunchArgument("planning_frame", default_value="camera_init"),
            DeclareLaunchArgument("max_linear_x", default_value="0.3"),
            DeclareLaunchArgument("max_angular_z", default_value="0.2"),
            DeclareLaunchArgument("lookahead_distance", default_value="0.6"),
            Node(
                package="scan_cmd_vel_follower",
                executable="scan_cmd_vel_follower",
                name="scan_cmd_vel_follower",
                output="screen",
                parameters=[
                    str(default_config),
                    {
                        "path_topic": path_topic,
                        "odom_topic": odom_topic,
                        "cmd_topic": cmd_topic,
                        "planning_frame": planning_frame,
                        "max_linear_x": max_linear_x,
                        "max_angular_z": max_angular_z,
                        "lookahead_distance": lookahead_distance,
                    },
                ],
            ),
        ]
    )
