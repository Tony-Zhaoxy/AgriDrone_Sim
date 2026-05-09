"""
simulation_greenhouse_flight.launch.py
============================
Full launch file for greenhouse navigation mission.

Starts (in order):
  1. RealSense D455 (color + depth + IMU, aligned depth)
  2. OpenVINS VIO
  3. VIO → PX4 bridge
  4. Occupancy grid builder
  5. Collision prevention CP bridge
  6. DWA local planner
  7. Safety monitor (independent watchdog)
  8. Mission executor

All nodes load parameters from config/mission.yaml.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('greenhouse_nav')
    mission_cfg = os.path.join(pkg, 'config', 'mission.yaml')
    

    # ── Launch arguments ──────────────────────────────────────────── #
    dist_arg = DeclareLaunchArgument(
        'mission_distance', default_value='10.0',
        description='Distance to fly forward in meters (10 or 15)')

    alt_arg = DeclareLaunchArgument(
        'takeoff_height', default_value='1.5',
        description='Flight altitude in meters AGL')

    # ── OpenVINS ──────────────────────────────────────────────────── #
    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='openvins',
        arguments=[ov_cfg],
        output='screen',
)

    # ── VIO → PX4 bridge ─────────────────────────────────────────── #
    vio_bridge_node = Node(
        package='greenhouse_nav',
        executable='vio_bridge',
        name='vio_bridge',
        output='screen',
    )

    # ── Occupancy grid ────────────────────────────────────────────── #
    occ_grid_node = Node(
        package='greenhouse_nav',
        executable='occupancy_grid',
        name='occupancy_grid_builder',
        parameters=[mission_cfg],
        output='screen',
    )

    # ── Collision Prevention CP bridge ────────────────────────────── #
    cp_node = Node(
        package='greenhouse_nav',
        executable='obstacle_avoidance',
        name='obstacle_avoidance_cp',
        parameters=[mission_cfg],
        output='screen',
    )

    # ── DWA planner ───────────────────────────────────────────────── #
    dwa_node = Node(
        package='greenhouse_nav',
        executable='dwa_planner',
        name='dwa_planner',
        parameters=[
            mission_cfg,
            {
                'mission_distance': LaunchConfiguration('mission_distance'),
                'takeoff_height':   LaunchConfiguration('takeoff_height'),
            }
        ],
        output='screen',
    )

    # ── Safety monitor ────────────────────────────────────────────── #
    safety_node = Node(
        package='greenhouse_nav',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[mission_cfg],
        output='screen',
    )

    # ── Mission executor (delayed 3s to let sensors stabilise) ───── #
    mission_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='greenhouse_nav',
                executable='mission_executor',
                name='mission_executor',
                parameters=[
                    mission_cfg,
                    {
                        'mission_distance': LaunchConfiguration('mission_distance'),
                        'takeoff_height':   LaunchConfiguration('takeoff_height'),
                    }
                ],
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        dist_arg,
        alt_arg,
        openvins_node,
        vio_bridge_node,
        occ_grid_node,
        cp_node,
        dwa_node,
        safety_node,
        mission_node,
    ])
