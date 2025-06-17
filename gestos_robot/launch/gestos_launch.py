from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='gestos_robot',
            executable='gestos_node',
            name='gestos_node',
            output='screen'
        ),
        Node(
            package='gestos_robot',
            executable='orden_subscriber',
            name='orden_subscriber',
            output='screen'
        ),
    ])
