from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('dummy_status'),
        'config',
        'dummy_settings.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='dummy_status',
            executable='dummy_status_node',
            name='dummy_status_node',
            parameters=[config]
        )
    ])