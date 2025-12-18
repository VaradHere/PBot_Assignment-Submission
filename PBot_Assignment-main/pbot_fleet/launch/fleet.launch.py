import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    
    # Launch arguments
    use_demo_orders_arg = DeclareLaunchArgument(
        'use_demo_orders',
        default_value='true',
        description='Load demo orders from waypoints.yaml (false for interactive-only mode)'
    )
    
    enable_interactive_arg = DeclareLaunchArgument(
        'enable_interactive',
        default_value='false',
        description='Enable interactive waypoint selector for RViz'
    )
    
    auto_dispatch_arg = DeclareLaunchArgument(
        'auto_dispatch',
        default_value='true',
        description='Enable automatic order dispatching'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    config_file = os.path.join(
        get_package_share_directory('pbot_fleet'),
        'config',
        'fleet_params.yaml'
    )
    
    # Fleet Manager Node
    fleet_manager_node = Node(
        package='pbot_fleet',
        executable='fleet_manager',
        name='fleet_manager',
        output='screen',
        parameters=[
            config_file,
            {
                'auto_dispatch': LaunchConfiguration('auto_dispatch'),
                'use_demo_orders': LaunchConfiguration('use_demo_orders'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        emulate_tty=True
    )
    
    # Interactive Waypoint Selector Node (for RViz-based order submission)
    interactive_waypoint_node = Node(
        package='pbot_fleet',
        executable='interactive_waypoint_selector',
        name='interactive_waypoint_selector',
        output='screen',
        parameters=[{
            'auto_submit': True,
            'marker_scale': 0.5,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
        condition=IfCondition(LaunchConfiguration('enable_interactive')),
        emulate_tty=True
    )
    
    return LaunchDescription([
        use_demo_orders_arg,
        enable_interactive_arg,
        auto_dispatch_arg,
        use_sim_time_arg,
        fleet_manager_node,
        interactive_waypoint_node
    ])
