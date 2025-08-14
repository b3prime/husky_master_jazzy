from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    # Expose arguments
    use_sim_arg = DeclareLaunchArgument(
        'use_sim',
        default_value = 'true',
        description = 'true -> runs Gazebo sim. false -> uses real robot.'
    )

    launch_platform_arg = DeclareLaunchArgument(
        'launch_platform',
        default_value = 'false',
        description = 'true -> launches serial connection to platform. false -> only launches sensors.'
    )
    
    robot_ns_arg = DeclareLaunchArgument(
        'robot_ns',
        default_value = 'a200_0000',
        description = 'namespace of robot. Used for formatting simulation topic names to match real hardware.'
    )

    setup_path_arg = DeclareLaunchArgument(
        'setup_path',
        default_value = '/etc/clearpath',
        description = 'path to robot.yaml file'
    )

    bringup_share_dir = FindPackageShare('brainbox_husky_bringup')
    clearpath_gz_share_dir = FindPackageShare('clearpath_gz')

    # Launch the simulation if use_sim argument is true
    sim_launch = PathJoinSubstitution([clearpath_gz_share_dir, 'launch', 'simulation.launch.py'])
    sim_include =   IncludeLaunchDescription(
                        PythonLaunchDescriptionSource([sim_launch]),
                        launch_arguments={
                        "setup_path": LaunchConfiguration("setup_path")
                        }.items(),
                        condition=IfCondition(LaunchConfiguration('use_sim')),
                    )

    # Launch the hardware platform if use_sim argument is false
    platform_launch_path = '/etc/clearpath/platform/launch/platform-service.launch.py'
    platform_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(platform_launch_path),
            condition = IfCondition(LaunchConfiguration('launch_platform')),
        )

    # Launch Vectornav driver if using real hardware
    # If using simulation, the topics are created by Gazebo,
    # And the relay_topics launch file relays them so their names align with the hardware topic names
    vn100_share = FindPackageShare('vectornav')
    vn100_launch_dir =  PathJoinSubstitution([vn100_share, 'launch', 'vectornav.launch.py'])
    vn100_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(vn100_launch_dir),
            condition = UnlessCondition(LaunchConfiguration('use_sim'))
            )
         
    
    # LaunchDescription for Ouster
    ouster_launch = PathJoinSubstitution([bringup_share_dir, 'launch', 'ouster.launch.py'])
    ouster_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([ouster_launch]),
        launch_arguments={
            'use_sim': LaunchConfiguration('use_sim'),
            'robot_ns': LaunchConfiguration('robot_ns'),
        }.items()
    )

    # LaunchDescription for UBlox
    ublox_launch =  PathJoinSubstitution([bringup_share_dir, 'launch', 'gnss_antenna.launch.py'])
    yaml_file = ( Path(get_package_share_directory("brainbox_husky_bringup"))
                    / "config" / "ublox.yaml"
                 )
    ublox_node = Node(
            package="ublox_gps",
            executable="ublox_gps_node",
            name = "ublox_gnss",
            parameters=[str(yaml_file)],
            output="screen",
            condition=UnlessCondition(LaunchConfiguration('use_sim')),
        )
    

    # the default Ouster laserscan extracts only one laser "row" from each scan
    # for effective mapping we need to consider multiple vertical bands
    # so we can avoid both low objects and taller objects
    pointcloud_to_laserscan_yaml = ( Path(get_package_share_directory("brainbox_husky_bringup")) / "config" / "pointcloud_to_laserscan.yaml"
                               )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="ouster_scan_band",
        output="screen",
        parameters = [ str(pointcloud_to_laserscan_yaml) ],
        remappings = [
            ("cloud_in", "/ouster/points"),
            ("scan", "/ouster/scan_multi")
        ]
    )

    # This remaps the simulation's topic names to match the hardware's native name (ex. /a200_0000/sensors/imu_0/imu -> /vectornav/imu').
    relay_topics_launch = PathJoinSubstitution([bringup_share_dir, 'launch', 'relay_topics.launch.py'])
    relay_topics_include = IncludeLaunchDescription(
                              PythonLaunchDescriptionSource([relay_topics_launch]),
                              condition=IfCondition(LaunchConfiguration('use_sim'))
                          )

    return LaunchDescription([
        setup_path_arg,
        use_sim_arg,
        robot_ns_arg,
        launch_platform_arg,
        
        #vn100_include,
        ublox_node,
        ouster_include,

        sim_include,
        platform_include,

        pointcloud_to_laserscan_node,

        relay_topics_include,
    ])

