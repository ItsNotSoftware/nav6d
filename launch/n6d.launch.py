#!/usr/bin/env python3
"""Launch file that brings up both the planner and the controller."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    planner_default = os.path.join(
        get_package_share_directory("nav6d"), "config", "n6d_planner.yaml"
    )
    controller_default = os.path.join(
        get_package_share_directory("nav6d"), "config", "n6d_controller.yaml"
    )

    planner_config = LaunchConfiguration("planner_config_file")
    controller_config = LaunchConfiguration("controller_config_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("planner_config_file", default_value=planner_default),
            DeclareLaunchArgument("controller_config_file", default_value=controller_default),
            Node(
                package="nav6d",
                executable="n6d_planner",
                name="n6d_planner",
                namespace="nav6d",
                output="screen",
                parameters=[planner_config],
            ),
            Node(
                package="nav6d",
                executable="n6d_controller",
                name="n6d_controller",
                namespace="nav6d",
                output="screen",
                parameters=[controller_config],
            ),
        ]
    )
