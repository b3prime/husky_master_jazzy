#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path

def generate_launch_description():

    pointcloud_topic_arg = DeclareLaunchArgument(
            'pointcloud_topic',
            default_value='/ouster/points',
            description="PointCloud2 topic for DLIO"
        )
    
    imu_topic_arg = DeclareLaunchArgument(
            'imu_topic',
            default_value='/ouster/imu',
            description='IMU topic for DLIO'
        )

    cfg = PathJoinSubstitution([FindPackageShare('path_planning'), 'config', 'ouster_slam_params.yaml'])
    
    dlio_pkg_share = get_package_share_directory('direct_lidar_inertial_odometry')
    dlio_launch_path = PathJoinSubstitution([dlio_pkg_share, 'launch', 'dlio.launch.py'])

    dlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(dlio_launch_path),
        launch_arguments={
            'rviz': 'false',
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'imu_topic': LaunchConfiguration('imu_topic'),
        }.items(),
    )

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

    return LaunchDescription([
        pointcloud_topic_arg,
        imu_topic_arg,
        slam, 
        dlio, 
        lm
        ])
