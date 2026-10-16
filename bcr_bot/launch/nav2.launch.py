import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_bcr = get_package_share_directory('bcr_bot')

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    # The map MUST be one recorded with slam_toolbox in the robot's real
    # environment. config/bcr_map.yaml is the Gazebo warehouse map and makes
    # AMCL localize against walls that do not exist on the physical robot.
    declare_map = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/maps/my_map.yaml'),
        description='Full path to the map yaml of the REAL environment '
                    '(save it with: ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map)')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_bcr, 'config', 'nav2_params.yaml'))
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='True')
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz', default_value='True',
        description='Set to False when running headless on the robot computer')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': map_yaml,
            'params_file': params_file,
        }.items()
    )

    rviz_launch_cmd = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=[
            '-d' + os.path.join(
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            )
        ]
    )

    # remapper.py (cmd_vel -> bcr_bot/cmd_vel) is only needed for the Gazebo
    # model. On the real robot esp32_driver listens on /cmd_vel directly.

    ld = LaunchDescription()

    ld.add_action(declare_map)
    ld.add_action(declare_params)
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_rviz)
    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_launch_cmd)

    return ld
