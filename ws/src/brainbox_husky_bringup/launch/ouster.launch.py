# Launches the Ouster-OS1 driver or equivalent parameter bridge. This gets called from the main launch file, husky_brainbox_bringup.launch.py

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import TextSubstitution, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():
    use_sim_launch_arg = DeclareLaunchArgument(
        'use_sim',
    )
    robot_ns_launch_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'a200_0000',
    )

    robot_ns = LaunchConfiguration('robot_ns')
    use_sim = LaunchConfiguration('use_sim')
    
    # spin up the parameter_bridge for the scan topic
    lidar_scan_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_scan_bridge',
            arguments= [
                PathJoinSubstitution([
                    TextSubstitution(
                        text='/a200_0000/sensors/lidar3d_0/scan'
                             '@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
                    ),
                ])
            ],    
            parameters=[{'lazy': False}],
            output='screen',
            remappings=[('/a200_0000/sensors/lidar3d_0/scan', '/ouster/scan')],
            condition=IfCondition(use_sim) # run bridge only in sim
    )

    # spin up the parameter_bridge for the points topic
    lidar_points_bridge = Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='lidar_points_bridge',
            arguments= [
                PathJoinSubstitution([
                    TextSubstitution(
                        text='/a200_0000/sensors/lidar3d_0/scan/points'
                             '@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked'
                    ),
                ])
            ],    
            parameters=[{'lazy': False}],
            output='screen',
            remappings=[('/a200_0000/sensors/lidar3d_0/scan/points', '/ouster/points')],
            condition=IfCondition(use_sim) # run bridge only in sim
    )

    # Real Ouster driver (hw-only)
    ouster_share = FindPackageShare('ouster_ros')
    ouster_launch = PathJoinSubstitution(
        [ouster_share, 'launch', 'driver.launch.py']
    )

    ouster_hw = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ouster_launch),
        #launch_arguments={}.items()
        condition = UnlessCondition(use_sim)
    )

    return LaunchDescription([
        use_sim_launch_arg,
        robot_ns_launch_arg,
        lidar_scan_bridge,
        lidar_points_bridge,
        ouster_hw,
    ])
