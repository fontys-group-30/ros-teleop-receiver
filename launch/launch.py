from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_receiver',
            executable='dynamic_tf_broadcaster',
            name='dynamic_tf_broadcaster',
            output='log'
        ),
        Node(
            package='teleop_receiver',
            executable='static_tf_broadcaster',
            name='static_tf_broadcaster',
            output='log'
        ),
        Node(
            package='teleop_receiver',
            executable='receiver_node',
            name='receiver_node',
            output='log'
        ),
    ])