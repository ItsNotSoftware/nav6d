#!/usr/bin/env python3
"""Launch file for the nav6d controllers (force + velocity)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


DEFAULT_CONTROLLER_TYPE = "velocity"  # Options: "velocity", "force"


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("nav6d")
    default_velocity_cfg = os.path.join(
        pkg_share, "config", "n6d_velocity_controller.yaml"
    )
    default_force_cfg = os.path.join(pkg_share, "config", "n6d_force_controller.yaml")

    controller_type = LaunchConfiguration("controller_type")
    velocity_config = LaunchConfiguration("velocity_config_file")
    force_config = LaunchConfiguration("force_config_file")

    def launch_setup(context, *args, **kwargs):
        requested = controller_type.perform(context).strip().lower()
        if requested not in {"velocity", "force"}:
            raise RuntimeError(
                f"Unsupported controller_type '{requested}'. Use 'velocity' or 'force'."
            )

        if requested == "velocity":
            executable = "n6d_velocity_controller"
            node_name = "n6d_velocity_controller"
            config_path = velocity_config.perform(context)
        else:
            executable = "n6d_force_controller"
            node_name = "n6d_force_controller"
            config_path = force_config.perform(context)

        return [
            Node(
                package="nav6d",
                executable=executable,
                name=node_name,
                namespace="nav6d",
                output="screen",
                parameters=[config_path],
            )
        ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_type", default_value=DEFAULT_CONTROLLER_TYPE
            ),
            DeclareLaunchArgument(
                "velocity_config_file", default_value=default_velocity_cfg
            ),
            DeclareLaunchArgument("force_config_file", default_value=default_force_cfg),
            OpaqueFunction(function=launch_setup),
        ]
    )
