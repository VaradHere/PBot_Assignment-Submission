import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    turtlebot3_navigation2_dir = get_package_share_directory('turtlebot3_navigation2')
    pbot_bringup_dir = get_package_share_directory('pbot_bringup')
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_file = LaunchConfiguration(
        'map',
        default=os.path.join(pbot_bringup_dir, 'maps', 'warehouse_map.yaml')
    )
    
    # TurtleBot3 Navigation2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_navigation2_dir, 'launch', 'navigation2.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_file
        }.items()
    )
    
    return LaunchDescription([
        nav2_launch
    ])
