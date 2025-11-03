#!/usr/bin/env python3
"""Launch file for the nav6d local planner."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def generate_launch_description() -> LaunchDescription:
    # Default parameter file bundled with the package.
    default_config = os.path.join(
        get_package_share_directory("nav6d"), "config", "n6d_planner.yaml"
    )

    config_file = LaunchConfiguration("config_file")

    return LaunchDescription(
        [
            # Allow callers to override the planner configuration at launch time.
            DeclareLaunchArgument("config_file", default_value=default_config),
            Node(
                package="nav6d",
                executable="n6d_planner",
                name="planner",
                namespace="nav6d",
                output="screen",
                # Pass the configuration path (and any overrides) straight into the node.
                parameters=[config_file],
            ),
        ]
    )
