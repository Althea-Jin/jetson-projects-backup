#!/usr/bin/env python3

"""
@file      scan_planner.launch.py
@brief     Launch SCAN-Planner together with its RViz2 visualization.
@author    juchunyu <juchunyu@qq.com>
@date      2026-07-23 20:00:01
@copyright Copyright (c) 2025-2026 Institute of Robotics Planning and Control (IRPC).
           All rights reserved.
This launch file starts the planner node and loads the packaged RViz configuration.
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("scan_planner"))
    rviz_config = package_share / "rviz" / "rviz.rviz"
    planner_config = package_share / "config" / "scan_mid360.yaml"
    use_rviz = LaunchConfiguration("use_rviz")
    cloud_topic = LaunchConfiguration("cloud_topic")
    odom_topic = LaunchConfiguration("odom_topic")
    global_path_topic = LaunchConfiguration("global_path_topic")
    planning_frame = LaunchConfiguration("planning_frame")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_rviz",
                default_value="true",
                description="Start RViz2 with the packaged SCAN-Planner configuration.",
            ),
            DeclareLaunchArgument(
                "cloud_topic",
                default_value="/robot_0/cloud_registered_world",
                description="PointCloud2 topic used as the planner obstacle input.",
            ),
            DeclareLaunchArgument(
                "odom_topic",
                default_value="/robot_0/odometry",
                description="Odometry topic used as the planner current pose input.",
            ),
            DeclareLaunchArgument(
                "global_path_topic",
                default_value="/scan_global_path",
                description="nav_msgs/Path topic used as the planner reference path input.",
            ),
            DeclareLaunchArgument(
                "planning_frame",
                default_value="world",
                description="Frame that cloud, odom, reference path, and visualization outputs must use.",
            ),
            Node(
                package="scan_planner",
                executable="motion_plan",
                name="scan_planner_interactive_node",
                parameters=[
                    str(planner_config),
                    {
                        "cloud_topic": cloud_topic,
                        "odom_topic": odom_topic,
                        "global_path_topic": global_path_topic,
                        "planning_frame": planning_frame,
                    },
                ],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", str(rviz_config)],
                output="screen",
                condition=IfCondition(use_rviz),
            ),
        ]
    )
