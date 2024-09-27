from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop-receiver',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster',
            output='log'
        ),
        Node(
            package='teleop-receiver',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='log'
        ),
        Node(
            package='teleop-receiver',
            executable='receiver-node',
            name='receiver-node',
            output='log'
        ),
    ])