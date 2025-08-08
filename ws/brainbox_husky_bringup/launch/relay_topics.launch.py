from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():

    imu_relay = Node(
        package='topic_tools',
        executable='relay',
        name='imu_relay',
        arguments=[
            '/a200_0000/sensors/imu_0/data',
            '/vectornav/imu'
        ],
        output='screen'
    )

    gps_relay = Node(
        package='topic_tools',
        executable='relay',
        name='gps_relay',
        arguments=[
            '/a200_0000/sensors/gps_0/fix',
            '/ublox_gnss/fix'
        ],
        output='screen'
    ) 

    return LaunchDescription([
        imu_relay,
        gps_relay,

    ])
