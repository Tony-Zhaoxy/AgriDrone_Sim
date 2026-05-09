# AgriNav — Drone Localization in Greenhouse

Autonomous multi-row greenhouse drone navigation stack for the Holybro X500 V2 + Intel RealSense D455 + Pixhawk 6X + Jetson Orin NX platform.

Operates fully GPS-denied using Visual-Inertial Odometry (VIO) and AprilTag/ArUco markers for row-end U-turns and landing.


## Flash new Jetpack to boost up Jetson
https://developer.nvidia.com/blog/nvidia-jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/

## Quick Start

```bash
# 1. Start the PX4 ↔ ROS 2 bridge
MicroXRCEAgent udp4 -p 8888

# 2. Bench test (no props, verify VIO + markers)
ros2 launch greenhouse_nav sensors_only.launch.py

# 3. Full autonomous mission
ros2 launch greenhouse_nav greenhouse_flight.launch.py
```

See **[greenhouse_nav/README.md](greenhouse_nav/README.md)** for the full installation guide, marker setup, calibration procedure, PX4 parameters, testing scenarios, and tuning guide.

## Repository Structure

```
AgriNav-Drone-localization-in-greenhouse/
└── greenhouse_nav/          # ROS 2 package — all source code, config, launch files
    ├── greenhouse_nav/      # Python nodes
    ├── config/              # YAML config (mission params, VIO, ORB_SLAM3)
    └── launch/              # Launch files
```

## Key Features

- **Dual VIO back-end**: OpenVINS (MSCKF, lightweight) or ORB_SLAM3 (loop-closure, multi-row drift correction)
- **Marker-guided navigation**: ArUco/AprilTag markers trigger mission init, row U-turns (left or right), and autonomous landing
- **Vectorised DWA planner**: EDT-based clearance + sector-scan recovery for reliable obstacle avoidance in narrow rows
- **Jetson Orin NX optimised**: All algorithms verified on aarch64 (no x86-only dependencies)

## License

MIT
