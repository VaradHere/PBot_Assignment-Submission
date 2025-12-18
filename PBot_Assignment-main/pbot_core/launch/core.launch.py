import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    
    # Get config file path
    config_file = os.path.join(
        get_package_share_directory('pbot_core'),
        'config',
        'core_params.yaml'
    )
    
    # AGV Supervisor Node
    agv_supervisor_node = Node(
        package='pbot_core',
        executable='agv_supervisor',
        name='agv_supervisor',
        output='screen',
        parameters=[config_file],
        emulate_tty=True
    )
    
    return LaunchDescription([
        agv_supervisor_node
    ])
