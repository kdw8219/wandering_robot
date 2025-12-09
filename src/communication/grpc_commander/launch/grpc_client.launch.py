from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    config = os.path.join(
        get_package_share_directory('grpc_commander'),
        'config',
        'grpc_settings.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='grpc_commander',
            executable='grpc_client_node',
            name='grpc_client_node',
            parameters=[config]
        )
    ])