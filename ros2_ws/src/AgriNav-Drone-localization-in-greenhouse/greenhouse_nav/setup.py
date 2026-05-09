from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'greenhouse_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy', 'scipy', 'opencv-python'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Vision-based indoor drone navigation for greenhouses',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vio_bridge        = greenhouse_nav.vio_bridge:main',
            'occupancy_grid    = greenhouse_nav.occupancy_grid:main',
            'dwa_planner       = greenhouse_nav.dwa_planner:main',
            'mission_executor  = greenhouse_nav.mission_executor:main',
            'safety_monitor    = greenhouse_nav.safety_monitor:main',
            'obstacle_avoidance= greenhouse_nav.obstacle_avoidance:main',
            'marker_detector   = greenhouse_nav.marker_detector:main',
            'orb_slam3_bridge  = greenhouse_nav.orb_slam3_bridge:main',
        ],
    },
)
