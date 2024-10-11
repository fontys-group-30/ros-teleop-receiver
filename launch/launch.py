from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # slam_toolbox_launch_dir = os.path.join(get_package_share_directory('slam_toolbox'), 'launch')
    rplidar_ros_launch_dir = os.path.join(get_package_share_directory('rplidar_ros'), 'launch')

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
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(os.path.join(slam_toolbox_launch_dir, 'online_async_launch.py')),
        #     launch_arguments={'use_sim_time': 'false'}.items()
        # ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_ros_launch_dir, 'rplidar.launch.py')),
        ),
    ])