"""
simulation_sensorlaunch.py
=======================
Launch ONLY sensors + VIO + bridge. No flight nodes.
Use this during bench testing and VIO calibration verification.

Steps:
  1. Place drone on desk (props OFF or removed)
  2. ros2 launch greenhouse_nav sensors_only.launch.py
  3. ros2 topic echo /ov_msckf/poseimu
  4. Move drone by hand — verify position tracks cleanly
  5. ros2 topic echo /fmu/out/vehicle_local_position
     → x/y/z should mirror OpenVINS output after VIO bridge
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg     = get_package_share_directory('greenhouse_nav')
    ov_cfg = '/home/user/ros2_ws/src/open_vins/config/rs_d455/estimator_config.yaml'

    openvins_node = Node(
        package='ov_msckf',
        executable='run_subscribe_msckf',
        name='openvins',
        arguments=[ov_cfg],
        output='screen',
)
    vio_bridge_node = Node(
        package='greenhouse_nav',
        executable='vio_bridge',
        name='vio_bridge',
        output='screen',
    )

    return LaunchDescription([
        openvins_node,
        vio_bridge_node,
    ])
