import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('greenhouse_nav')
    mission_cfg = os.path.join(pkg, 'config', 'mission.yaml')

    dist_arg = DeclareLaunchArgument(
        'mission_distance',
        default_value='10.0',
        description='Distance to fly forward in meters'
    )

    alt_arg = DeclareLaunchArgument(
        'takeoff_height',
        default_value='1.5',
        description='Flight altitude in meters'
    )

    mission_delay_arg = DeclareLaunchArgument(
        'mission_start_delay',
        default_value='8.0',
        description='Delay before starting mission_executor'
    )

    occ_grid_node = Node(
        package='greenhouse_nav',
        executable='occupancy_grid',
        name='occupancy_grid_builder',
        parameters=[mission_cfg],
        output='screen',
    )

    avoidance_node = Node(
        package='greenhouse_nav',
        executable='obstacle_avoidance',
        name='obstacle_avoidance',
        parameters=[mission_cfg],
        output='screen',
    )

    dwa_node = Node(
        package='greenhouse_nav',
        executable='dwa_planner',
        name='dwa_planner',
        parameters=[
            mission_cfg,
            {
                'takeoff_height': LaunchConfiguration('takeoff_height'),
            }
        ],
        output='screen',
    )

    safety_node = Node(
        package='greenhouse_nav',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[mission_cfg],
        output='screen',
    )

    vio_bridge_node = Node(
        package='greenhouse_nav',
        executable='vio_bridge',
        name='vio_bridge',
        output='screen',
    )

    mission_node = TimerAction(
        period=LaunchConfiguration('mission_start_delay'),
        actions=[
            Node(
                package='greenhouse_nav',
                executable='mission_executor',
                name='mission_executor',
                parameters=[
                    mission_cfg,
                    {
                        'mission_distance': LaunchConfiguration('mission_distance'),
                        'takeoff_height': LaunchConfiguration('takeoff_height'),
                    }
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        dist_arg,
        alt_arg,
        mission_delay_arg,
        vio_bridge_node,
        occ_grid_node,
        avoidance_node,
        dwa_node,
        safety_node,
        mission_node,
    ])