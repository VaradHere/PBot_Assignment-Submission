import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    
    # Package directories
    bringup_dir = get_package_share_directory('pbot_bringup')
    core_dir = get_package_share_directory('pbot_core')
    fleet_dir = get_package_share_directory('pbot_fleet')
    gripper_dir = get_package_share_directory('pbot_gripper')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_demo_orders = LaunchConfiguration('use_demo_orders', default='true')
    enable_interactive = LaunchConfiguration('enable_interactive', default='false')
    
    # 1. Simulation (Gazebo + Robot)
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'turtlebot3_pbot.launch.py')
        )
    )
    
    # 2. Navigation (Nav2 with map)
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'nav2_bringup.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # 3. Gripper Simulator
    gripper_node = Node(
        package='pbot_gripper',
        executable='simulated_gripper',
        name='simulated_gripper',
        output='screen',
        emulate_tty=True
    )
    
    # 4. AGV Core (Supervisor)
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_dir, 'launch', 'core.launch.py')
        )
    )
    
    # 5. Fleet Manager (with interactive waypoint selector)
    fleet_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fleet_dir, 'launch', 'fleet_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'use_demo_orders': use_demo_orders,
            'enable_interactive': enable_interactive,
        }.items()
    )
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation time'),
        DeclareLaunchArgument('use_demo_orders', default_value='true',
                              description='Load demo orders from YAML (false for interactive-only)'),
        DeclareLaunchArgument('enable_interactive', default_value='false',
                              description='Enable interactive RViz waypoint selector'),

        # Launch nodes and subsystems
        sim_launch,
        nav_launch,
        gripper_node,
        core_launch,
        fleet_launch,
    ])
