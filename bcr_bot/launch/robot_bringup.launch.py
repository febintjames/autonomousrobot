"""
Bring up the PHYSICAL robot: lidar + base_link->laser TF + ESP32 motor/odometry driver.

Run this first, then mapping.launch.py (to build a map) or nav2.launch.py (to navigate).

  ros2 launch bcr_bot robot_bringup.launch.py
  ros2 launch bcr_bot robot_bringup.launch.py lidar_port:=/dev/rplidar esp32_port:=/dev/esp32
  ros2 launch bcr_bot robot_bringup.launch.py odom_source:=laser   # use rf2o instead of encoders

Exactly ONE node publishes the odom -> base_link TF and /odom:
  odom_source:=wheel (default)  esp32_driver from encoders, rf2o not started
  odom_source:=laser            rf2o from lidar scans, esp32_driver odom moved to /odom_wheel (no TF)
Two publishers of the same TF make the robot pose jump back and forth between them.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    odom_source = LaunchConfiguration('odom_source')
    use_wheel_odom = PythonExpression(["'", odom_source, "' == 'wheel'"])
    use_laser_odom = PythonExpression(["'", odom_source, "' == 'laser'"])

    args = [
        DeclareLaunchArgument('odom_source', default_value='wheel',
                              description="'wheel' (ESP32 encoders) or 'laser' (rf2o)"),
        # USB order can swap after a reboot. Prefer stable names, e.g.
        # /dev/serial/by-id/... or the udev symlinks from sllidar_ros2/scripts.
        DeclareLaunchArgument('lidar_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('lidar_baudrate', default_value='115200',
                              description='A1/A2M8: 115200, C1: 460800, A2M12/A3: 256000'),
        DeclareLaunchArgument('esp32_port', default_value='/dev/ttyUSB1'),
        # Where the lidar centre sits relative to base_link (centre between the
        # drive wheels, on the floor). Measure it on the robot, in metres.
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.15'),
        # Put a box 50 cm in FRONT of the robot and look at /scan in RViz
        # (fixed frame base_link). If the box shows up BEHIND the robot, use 3.14159.
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
    ]

    lidar = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': LaunchConfiguration('lidar_port'),
            'serial_baudrate': LaunchConfiguration('lidar_baudrate'),
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
        }],
    )

    laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_laser',
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--pitch', '0', '--roll', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    esp32 = Node(
        package='my_python_pkg',
        executable='esp32_driver',
        name='esp32_teleop_odometry_node',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('esp32_port'),
            'publish_tf': use_wheel_odom,
            'odom_topic': PythonExpression(
                ["'/odom' if '", odom_source, "' == 'wheel' else '/odom_wheel'"]),
            # Calibrate these on the real robot with `ros2 run my_python_pkg odom_check`
            # (see ROBOT_NAV_GUIDE.md section 3):
            'wheel_radius': 0.0425,
            'wheel_separation': 0.34,
            'ticks_per_revolution': 537.6,
            'linear_scale': 1.0,
            'invert_left_encoder': False,
            'invert_right_encoder': False,
            # Keep equal to velocity_smoother max_velocity in nav2_params.yaml
            'max_linear_speed': 0.18,
            'max_angular_speed': 0.50,
        }],
    )

    rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        condition=IfCondition(use_laser_odom),
        parameters=[{
            'laser_scan_topic': '/scan',
            'odom_topic': '/odom',
            'publish_tf': True,
            'base_frame_id': 'base_link',
            'odom_frame_id': 'odom',
            'init_pose_from_topic': '',
            'freq': 10.0,
        }],
    )

    return LaunchDescription(args + [lidar, laser_tf, esp32, rf2o])
