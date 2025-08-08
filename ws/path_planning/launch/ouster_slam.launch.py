#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from pathlib import Path

def generate_launch_description():
    cfg = PathJoinSubstitution([FindPackageShare('path_planning'), 'config', 'ouster_slam_params.yaml'])

    slam = Node(
        package = 'slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[cfg],
        remappings=[('/scan', '/ouster/scan_multi')]
    )

    lm = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{"autostart": True, "node_names":['slam_toolbox']}]
    )

    return LaunchDescription([slam, lm])
