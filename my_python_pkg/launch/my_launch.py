from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pub_node = Node(
        package='my_python_pkg',
        executable='pub_node',
        name='publisher_node'
    )

    sub_node = Node(
        package='my_python_pkg',
        executable='sub_node',
        name='subscriber_node'
    )


    return LaunchDescription([
        pub_node,
        sub_node
    ])
