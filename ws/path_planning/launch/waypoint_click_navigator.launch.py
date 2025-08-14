#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('goal_timeout_sec', default_value='0.0',
                              description='Timeout (seconds) for a goal; 0 = unlimited'),
        DeclareLaunchArgument('face_goal', default_value='true',
                              description='true -> robot will face the goal from its current position'),
        DeclareLaunchArgument('default_yaw_deg', default_value='0.0',
                              description='Used when face_goal:=false; heading at the goal in degrees'),

        Node(
            package='path_planning',
            executable='waypoint_click_navigator.py',
            name='waypoint_click_navigator',
            output='screen',
            parameters=[{
                'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
                'face_goal': LaunchConfiguration('face_goal'),
                'default_yaw_deg': LaunchConfiguration('default_yaw_deg'),
            }],
        ),
    ])

