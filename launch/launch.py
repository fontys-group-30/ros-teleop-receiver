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
            executable='receiver_node',
            name='receiver_node',
            output='log'
        ),
        Node(
            package='slam_toolbox',
            executable='online_async_launch.py',
            name='slam_toolbox',
            output='screen',
            parameters=[{'use_sim_time': False}]
        ),
    ])