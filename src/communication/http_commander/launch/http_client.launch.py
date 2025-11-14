from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('http_commander'),
        'config',
        'http_settings.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='http_commander',
            executable='http_client_node',
            name='http_client_node',
            parameters=[config]
        )
    ])