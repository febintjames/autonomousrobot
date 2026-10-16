"""
Start the web app (needs Nav2 running: ros2 launch bcr_bot nav2.launch.py).

  ros2 launch table_nav_web table_nav_web.launch.py
  ros2 launch table_nav_web table_nav_web.launch.py port:=8000

Then open http://<robot-ip>:8080 on a phone/laptop on the same Wi-Fi.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('locations_file', default_value='~/.ros/table_locations.yaml'),
        Node(
            package='table_nav_web',
            executable='web_server',
            name='table_nav_web',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'locations_file': LaunchConfiguration('locations_file'),
                'map_frame': 'map',
                'base_frame': 'base_link',
            }],
        ),
    ])
