import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg         = get_package_share_directory('greenhouse_nav')
    mission_cfg = os.path.join(pkg, 'config', 'mission.yaml')
    vio_cfg     = os.path.join(pkg, 'config', 'd455_vio.yaml')
    orb_cfg     = os.path.join(pkg, 'config', 'd455_orbslam3.yaml')
    rviz_cfg    = os.path.join(pkg, 'config', 'openvins_debug.rviz')

    args = [
        DeclareLaunchArgument('vio_backend', default_value='openvins'),
        DeclareLaunchArgument('orb_vocab', default_value='/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt'),
        DeclareLaunchArgument('orb_pkg', default_value='orb_slam3_ros2'),
        DeclareLaunchArgument('orb_exe', default_value='stereo_inertial'),
    ]

    use_openvins = IfCondition(
        PythonExpression(["'", LaunchConfiguration('vio_backend'), "' == 'openvins'"]))
    use_orbslam3 = IfCondition(
        PythonExpression(["'", LaunchConfiguration('vio_backend'), "' == 'orbslam3'"]))

    rs_launch = PathJoinSubstitution([
        FindPackageShare('realsense2_camera'),
        'launch',
        'rs_launch.py'
    ])

    d455_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rs_launch),
        launch_arguments={
            'camera_namespace': 'd455',
            'camera_name': 'd455',
            'enable_color': 'true',
            'rgb_camera.color_profile': '848x480x30',
            'enable_depth': 'false',
            'pointcloud.enable': 'false',
            'align_depth.enable': 'false',
            'enable_infra1': 'true',
            'enable_infra2': 'true',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'unite_imu_method': '2',
            'gyro_fps': '200',
            'accel_fps': '200',
            'emitter_enabled': '1',
            'initial_reset': 'true',
        }.items()
    )

    openvins_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                condition=use_openvins,
                package='ov_msckf',
                executable='run_subscribe_msckf',
                name='openvins',
                arguments=[vio_cfg],
                output='screen',
            )
        ],
    )

    orbslam3_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                condition=use_orbslam3,
                package=LaunchConfiguration('orb_pkg'),
                executable=LaunchConfiguration('orb_exe'),
                name='orb_slam3',
                parameters=[{
                    'vocabulary_file_path': LaunchConfiguration('orb_vocab'),
                    'settings_file_path': orb_cfg,
                }],
                remappings=[
                    ('/camera/left',  '/d455/d455/infra1/image_rect_raw'),
                    ('/camera/right', '/d455/d455/infra2/image_rect_raw'),
                    ('/imu',          '/d455/d455/imu'),
                ],
                output='screen',
            )
        ],
    )

    vio_bridge_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                condition=use_openvins,
                package='greenhouse_nav',
                executable='vio_bridge',
                name='vio_bridge',
                output='screen',
            )
        ],
    )

    orb_bridge_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                condition=use_orbslam3,
                package='greenhouse_nav',
                executable='orb_slam3_bridge',
                name='orb_slam3_bridge',
                parameters=[mission_cfg],
                output='screen',
            )
        ],
    )

    marker_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='greenhouse_nav',
                executable='marker_detector',
                name='marker_detector',
                parameters=[mission_cfg],
                remappings=[
                    ('/d455/color/image_raw', '/d455/d455/color/image_raw'),
                ],
                output='screen',
            )
        ],
    )

    rviz_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_cfg],
                output='screen',
            )
        ],
    )

    return LaunchDescription(
        args + [
            d455_launch,
            openvins_node,
            orbslam3_node,
            vio_bridge_node,
            orb_bridge_node,
            marker_node,
            rviz_node,
        ]
    )
