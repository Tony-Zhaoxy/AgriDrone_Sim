# greenhouse_nav

Vision-based autonomous multi-row greenhouse drone navigation.
**Hardware:** Holybro X500 V2 · Intel RealSense D455 · Pixhawk 6X · Jetson Orin NX 16 GB · Optical Flow & Distance Sensor · DroneCAN RM3100 Compass

---

## What This Package Does

Implements a complete autonomous navigation stack for a drone to fly multiple greenhouse rows without GPS:

1. **Wait** for the VIO-init AprilTag/ArUco marker (ID 0) to confirm the drone is in position and localisation is ready
2. **Fly forward** through a row, avoiding obstacles in real-time with a DWA local planner
3. **U-turn** at the row end when a left/right marker (ID 1 or 2) is detected — lateral shift to the next row, then resume forward flight
4. **Repeat** for as many rows as needed; U-turn direction is encoded in the marker so left/right turns can be mixed
5. **Stop and land** when the final marker (ID 10) is detected

All navigation is GPS-denied, using Visual-Inertial Odometry from the D455 stereo IR cameras + IMU.

---

## Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                               Jetson Orin NX                                  │
│                                                                                │
│  D455 IR L+R + IMU ──→ OpenVINS (stereo MSCKF)  ─┐                           │
│                    └──→ ORB_SLAM3 (stereo-inertial)┤→ vio/orb bridge → PX4 EKF2│
│                                                    │                           │
│  D455 Color ─────────→ marker_detector ────────────→ /marker/event            │
│                                                    │                           │
│  D455 Depth ─────────→ occupancy_grid ─────────────→ dwa_planner → PX4 vel   │
│                    └──→ obstacle_avoidance ─────────→ PX4 CP (backup)         │
│                                                                                │
│  mission_executor ←── /marker/event                                            │
│  (WAIT_INIT → FORWARD → UTURN → FORWARD → … → LAND)                           │
│       └──────────────→ /mission/goal ──────────────→ dwa_planner              │
│                                                                                │
│  safety_monitor  (independent watchdog — triggers NAV_LAND on fault)          │
└───────────────────────────────────────────────────────────────────────────────┘
                                │ uXRCE-DDS UDP
┌───────────────────────────────────────────────────────────────────────────────┐
│                          Pixhawk 6X  (PX4 firmware)                           │
│    EKF2 ← VIO pose  │  CP ← ObstacleDistance  │  Motor/ESC outputs           │
└───────────────────────────────────────────────────────────────────────────────┘
```

### VIO back-end — choose one, not both

| Back-end | When to use |
|---|---|
| **OpenVINS** (default) | Single short row, simpler setup, lower CPU |
| **ORB_SLAM3** | Multi-row missions, needs loop-closure to bound drift; recovers from tracking loss |

### Node Summary

| Node | Role |
|---|---|
| `vio_bridge` | Relays OpenVINS pose to PX4 EKF2 (ENU→NED, finite-diff velocity) |
| `orb_slam3_bridge` | Same as above but reads ORB_SLAM3 pose; handles jump/tracking-loss |
| `marker_detector` | Detects ArUco/AprilTag markers, publishes semantic events on `/marker/event` |
| `occupancy_grid` | Projects D455 depth into a 2D bird's-eye obstacle map with EDT |
| `dwa_planner` | Vectorised DWA local planner — computes safe velocity commands |
| `obstacle_avoidance` | Feeds PX4's Collision Prevention as a hardware safety net |
| `mission_executor` | State machine: WAIT_INIT → FORWARD → UTURN_ROT1/MOVE/ROT2 → LAND |
| `safety_monitor` | Independent watchdog — lands drone if VIO lost or altitude exceeded |

---

## Directory Structure

```
greenhouse_nav/
├── greenhouse_nav/
│   ├── __init__.py
│   ├── vio_bridge.py          # OpenVINS → PX4 bridge
│   ├── orb_slam3_bridge.py    # ORB_SLAM3 → PX4 bridge (drop-in replacement)
│   ├── marker_detector.py     # ArUco/AprilTag detector → /marker/event
│   ├── occupancy_grid.py      # Depth → 2D obstacle map
│   ├── dwa_planner.py         # DWA local planner (vectorised + EDT)
│   ├── mission_executor.py    # Multi-row flight state machine
│   ├── safety_monitor.py      # Independent watchdog
│   ├── obstacle_avoidance.py  # PX4 Collision Prevention feeder
│   └── set_px4_params.py      # One-time PX4 parameter setup helper
├── config/
│   ├── d455_vio.yaml          # OpenVINS stereo-inertial config
│   ├── d455_orbslam3.yaml     # ORB_SLAM3 stereo-inertial config
│   └── mission.yaml           # All tunable parameters for every node
├── launch/
│   ├── greenhouse_flight.launch.py   # Full autonomous mission
│   ├── sensors_only.launch.py        # Bench test — no flight nodes
│   ├── simulation_greenhouse_flight.launch.py
│   ├── simulation_sensor.launch.py
│   └── simulation_mission_only.launch.py
├── package.xml
├── setup.py
└── setup.cfg
```

---

## Prerequisites

| Requirement | Version |
|---|---|
| Jetson Orin NX | JetPack 6.x (Ubuntu 22.04, aarch64) |
| ROS 2 | Humble |
| PX4 firmware | v1.14+ on Pixhawk 6X |
| Python | 3.10+ |
| OpenCV | 4.5+ (with contrib for ArUco) |

---

## Installation

### Step 1 — ROS 2 Humble

```bash
sudo apt install -y \
  ros-humble-desktop \
  python3-colcon-common-extensions \
  python3-rosdep

source /opt/ros/humble/setup.bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

### Step 2 — RealSense SDK + ROS 2 wrapper

```bash
sudo apt install -y ros-humble-realsense2-camera ros-humble-realsense2-description

# Verify D455 is detected
rs-enumerate-devices | grep D455
```

### Step 3 — Python dependencies

```bash
sudo apt install -y python3-numpy python3-scipy python3-opencv
# opencv-contrib (needed for ArUco on some builds)
pip3 install opencv-contrib-python --break-system-packages
```

### Step 4 — uXRCE-DDS Agent (Jetson ↔ PX4)

```bash
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j$(nproc) && sudo make install
```

### Step 5 — px4_msgs (must match your PX4 firmware version)

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone -b release/1.14 https://github.com/PX4/px4_msgs.git

cd ~/ros2_ws
colcon build --packages-select px4_msgs
source install/setup.bash
```

### Step 6 — VIO back-end (choose one)

#### Option A — OpenVINS (default, simpler)

```bash
cd ~/ros2_ws/src
git clone https://github.com/rpng/open_vins.git

sudo apt install -y libeigen3-dev libopencv-dev ros-humble-cv-bridge

cd ~/ros2_ws
colcon build --packages-select ov_core ov_init ov_msckf \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
```

#### Option B — ORB_SLAM3 (recommended for multi-row, loop-closure)

```bash
# Native build
cd ~/ros2_ws/src
git clone https://github.com/Mechazo11/ros2_orb_slam3.git
# Follow the repo's own build instructions (Pangolin + ORB_SLAM3 core first)

# OR use the Docker wrapper (easier on Jetson):
git clone https://github.com/suchetanrs/ORB-SLAM3-ROS2-Docker.git
cd ORB-SLAM3-ROS2-Docker && docker compose up

# Default vocabulary path expected by the launch file:
# /opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
# Adjust orb_vocab launch argument if your path differs.
```

### Step 7 — Build greenhouse_nav

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/realsense_ws/install/setup.bash
source ~/workspace/Agriculture_Drone/drone/vla_ws/install/setup.bash
source ~/ws_openvins/install/setup.bash
source ~/AgriNav/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y   # installs numpy/scipy/opencv
colcon build --packages-select greenhouse_nav
source install/setup.bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Marker Setup

The system uses ArUco (or AprilTag) markers for three purposes:

| Marker ID | Event | Purpose |
|---|---|---|
| **0** | `MARKER_INIT` | VIO confirmed, start the mission |
| **1** | `MARKER_UTURN_LEFT` | End of row — U-turn rotating LEFT |
| **2** | `MARKER_UTURN_RIGHT` | End of row — U-turn rotating RIGHT |
| **10** | `MARKER_LAND` | Stop flying and land |

### Print markers

1. Generate markers from the **DICT_4X4_50** dictionary (default) using OpenCV or [this online tool](https://chev.me/arucogen/)
2. Print at exactly **15 cm × 15 cm** physical size (adjust `marker_size_m` in `mission.yaml` if different)
3. Mount on flat, rigid boards — avoid wrinkled or glossy paper

### Place markers

```
Row 1 start          Row 1 end             Row 2 end
    [ID 0]  ═══════════ [ID 1 or 2] ═══════════ [ID 1 or 2]
    (face camera        (face approaching          ...
     at take-off)        drone)              [ID 10] (landing zone)
```

- **ID 0** faces the camera at the drone's starting/take-off position
- **Row-end markers** face the approaching drone, placed at the far wall or a pole
- **Landing marker (ID 10)** placed flat on the ground at the final landing point, or vertical facing the approach direction
- Markers must be visible from **≥ 0.5 m and ≤ 4 m** distance
- Disable the D455 IR projector before flight to avoid interference with IR camera feature tracking:

```bash
ros2 param set /camera/camera depth_module.emitter_enabled 0
```

### Change marker dictionary

To use **AprilTag** markers instead of ArUco, update `mission.yaml`:

```yaml
marker_detector:
  ros__parameters:
    aruco_dict: DICT_APRILTAG_36h11   # or DICT_APRILTAG_16h5
```

---

## Camera-IMU Calibration

**Must be done before first flight.** All config files contain factory-nominal values only — wrong extrinsics are the #1 cause of VIO drift and failed initialisation.

### What gets calibrated

| Config file | Values to replace |
|---|---|
| `config/d455_vio.yaml` | `cam0/cam1_intrinsics`, `cam0/cam1_distortion_coeffs`, `T_cam0_imu`, `T_cam1_imu` |
| `config/d455_orbslam3.yaml` | `Camera.fx/fy/cx/cy`, `Camera2.*`, `Stereo.T_c1_c2`, `IMU.T_b_c1`, `Camera.bf` |
| `config/mission.yaml` | `fx/fy/cx/cy` in `occupancy_grid_builder` and `marker_detector` (colour camera) |

---

### Phase 0 — Install Kalibr

Docker is the easiest option (avoids ROS version conflicts):

```bash
docker pull ghcr.io/ethz-asl/kalibr:latest
```

Native build: see <https://github.com/ethz-asl/kalibr/wiki/installation>

---

### Phase 0b — Print and measure an AprilGrid target

```bash
# Generate a 6×6 AprilGrid with 3 cm tags
kalibr_create_target_pdf \
    --type apriltag \
    --nx 6 --ny 6 \
    --tsize 0.03 \
    --tspace 0.3 \
    --output april_grid_6x6_3cm.pdf
```

Print at **100% scale** (no "fit to page"). After printing, measure the actual tag side length with calipers and create `april_grid.yaml`:

```yaml
# april_grid.yaml
target_type: aprilgrid
tagCols:    6
tagRows:    6
tagSize:    0.068     # ← replace with your measured value in metres
tagSpacing: 0.294       # ratio of gap to tag size (0.3 = 9 mm gap for 30 mm tags)
```

Mount the target flat on a rigid board — avoid glossy paper and wrinkles.

---

### Phase 1 — Stereo Camera Calibration

#### 1a. Start the D455 and disable the IR projector

```bash

source /opt/ros/humble/setup.bash
source ~/workspace/realsense_ws/install/setup.bash
source ~/workspace/Agriculture_Drone/drone/vla_ws/install/setup.bash
source ~/ws_openvins/install/setup.bash
source ~/AgriNav/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH


# Terminal 1
# ros2 launch greenhouse_nav sensors_only.launch.py
ros2 launch realsense2_camera rs_launch.py \
  camera_namespace:=camera camera_name:=camera \
  enable_color:=true rgb_camera.color_profile:=848x480x30 \
  enable_depth:=false pointcloud.enable:=false align_depth.enable:=false \
  enable_infra1:=true enable_infra2:=true \
  depth_module.infra_profile:=848x480x30 \
  enable_gyro:=true enable_accel:=true unite_imu_method:=2 \
  gyro_fps:=200 accel_fps:=200 \
  initial_reset:=true

# Terminal 2 — MUST disable IR dot projector before recording
ros2 param set /camera/camera depth_module.emitter_enabled 0

# Verify images look sharp (no blur)
ros2 run rqt_image_view rqt_image_view /camera/camera/infra1/image_rect_raw
```

#### 1b. Record the stereo bag

```bash
ros2 bag record \
    /camera/camera/infra1/image_rect_raw \
    /camera/camera/infra2/image_rect_raw \
    -o calib_stereo
```

**How to move:** hold the AprilGrid still, move the camera slowly across it. Cover all corners of the field of view. Aim for ~60–120 seconds of data. Slow, blur-free motion only.

#### 1c. Convert bag format (ROS 2 → ROS 1)

Kalibr requires a ROS 1 `.bag` file:

```bash
pip install rosbags
rosbags-convert --src calib_stereo --dst calib_stereo.bag
```

#### 1d. Run Kalibr stereo calibration

```bash

FOLDER=/home/scidrone/Documents/data_docker
xhost +local:root
sudo docker run -it -e "DISPLAY" -e "QT_X11_NO_MITSHM=1" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    -v "$FOLDER:/data" kalibr
    
    
sudo docker run --rm -it \
  -v "/home/scidrone/Documents/data_docker/data:/targets" \
  -v "/home/scidrone/Documents/AgriNav-Drone-localization-in-greenhouse/greenhouse_nav:/bags" \
  kalibr \
  /bin/bash -lc '
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    rosrun kalibr kalibr_calibrate_cameras \
      --target /targets/april_grid.yaml \
      --bag /bags/calib_stereo.bag \
      --models pinhole-radtan pinhole-radtan \
      --topics /camera/camera/infra1/image_rect_raw /camera/camera/infra2/image_rect_raw \
      --dont-show-report
  '
```

**Outputs:** `camchain.yaml` and `report-cam-*.pdf`.

Check the PDF — reprojection error should be:
- **< 0.3 px** — excellent
- **0.3–0.7 px** — acceptable
- **> 1.0 px** — redo calibration (usually motion blur or bad target print)

---

### Phase 2 — Camera-IMU Calibration

#### 2a. Create `imu.yaml`

```yaml
# imu.yaml  (D455 BMI055 — datasheet starting values)
rostopic:    /camera/camera/imu/data
update_rate: 200   # Hz

accelerometer_noise_density:   2.0e-3   # m/s²/√Hz
accelerometer_random_walk:     3.0e-5   # m/s³/√Hz
gyroscope_noise_density:       1.6e-4   # rad/s/√Hz
gyroscope_random_walk:         2.2e-6   # rad/s²/√Hz
```

> **Optional — improve IMU noise values with Allan variance:**
> Record 2 hours of stationary IMU data, then run `allan_variance_ros` or `imu_utils`.
> Skip for a first calibration; datasheet values work well enough.

#### 2b. Record the camera-IMU bag

```bash
source /opt/ros/humble/setup.bash
source ~/workspace/realsense_ws/install/setup.bash
source ~/workspace/Agriculture_Drone/drone/vla_ws/install/setup.bash
source ~/ws_openvins/install/setup.bash
source ~/AgriNav/install/setup.bash
export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

ros2 bag record \
    /camera/camera/infra1/image_rect_raw \
    /camera/camera/infra2/image_rect_raw \
    /camera/camera/imu \
    -o calib_imu
```

**How to move — this is the most important step:**
- Excite **all 6 axes**: pitch, roll, yaw, +X, +Y, +Z translation
- Move **slowly and smoothly** — no sudden jerks, no vibration
- Keep the AprilGrid fully visible throughout (fix it to a wall)
- Aim for **120–180 seconds** of data
- End with the board fully in frame (Kalibr initialises better this way)

#### 2c. Convert and run IMU-camera calibration

```bash
rosbags-convert --src calib_imu --dst calib_imu.bag

ros2 bag play calib_imu
ros2 bag info calib_imu


sudo docker run --rm -it \
  -v "/home/scidrone/Documents/data_docker/data:/targets" \
  -v "/home/scidrone/Documents/AgriNav-Drone-localization-in-greenhouse/greenhouse_nav:/bags" \
  kalibr \
  /bin/bash -lc '
    source /opt/ros/noetic/setup.bash
    source /catkin_ws/devel/setup.bash
    rosrun kalibr kalibr_calibrate_imu_camera \
      --target /targets/april_grid.yaml \
      --bag /bags/calib_imu.bag \
      --cam /targets/camchain.yaml \
      --imu /targets/imu.yaml \
      --dont-show-report
  '
```

**Outputs:** `camchain-imucam.yaml` and `report-imucam-*.pdf`.

---

### Phase 3 — Transfer Results to Config Files

#### Read the Kalibr output

`camchain-imucam.yaml` contains:

```yaml
cam0:
  camera_model: pinhole
  intrinsics: [fx, fy, cx, cy]       # ← copy to d455_vio.yaml cam0_intrinsics
  distortion_model: radtan
  distortion_coeffs: [k1, k2, p1, p2]
  T_cam_imu:                         # ← copy to d455_vio.yaml T_cam0_imu
  - [r00, r01, r02, tx]
  ...
cam1:
  intrinsics: [fx, fy, cx, cy]       # ← copy to d455_vio.yaml cam1_intrinsics
  T_cam_imu: ...                     # ← copy to d455_vio.yaml T_cam1_imu
  T_cn_cnm1:                         # right-to-left extrinsic
  - [1.0, 0.0, 0.0, -0.09505]       # ← copy to d455_orbslam3.yaml Stereo.T_c1_c2
  ...
```

#### Update `config/d455_vio.yaml`

Replace the four `# REPLACE with Kalibr output` blocks:

```yaml
cam0_intrinsics:        [ fx0, fy0, cx0, cy0 ]   # from cam0.intrinsics
cam0_distortion_coeffs: [ k1, k2, p1, p2 ]       # from cam0.distortion_coeffs
cam0_distortion_model:  "radtan"

T_cam0_imu:                                       # from cam0.T_cam_imu
  - [ r00, r01, r02, tx ]
  - [ r10, r11, r12, ty ]
  - [ r20, r21, r22, tz ]
  - [ 0.0, 0.0, 0.0, 1.0 ]

cam1_intrinsics:        [ fx1, fy1, cx1, cy1 ]   # from cam1.intrinsics
cam1_distortion_coeffs: [ k1, k2, p1, p2 ]

T_cam1_imu:                                       # from cam1.T_cam_imu
  - [ ... ]
```

#### Update `config/d455_orbslam3.yaml`

```yaml
# Left camera (cam0):
Camera.fx: <fx0>
Camera.fy: <fy0>
Camera.cx: <cx0>
Camera.cy: <cy0>
Camera.k1: <k1>   Camera.k2: <k2>
Camera.p1: <p1>   Camera.p2: <p2>

# Right camera (cam1):
Camera2.fx: <fx1>  ...

# Stereo baseline — from T_cn_cnm1 translation x component:
Camera.bf: <fx0 * |tx|>   # e.g. 379.12 * 0.09505 = 36.04
Stereo.b:  <|tx|>          # absolute value of translation x

# Stereo extrinsic — copy T_cn_cnm1 directly:
Stereo.T_c1_c2: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: [ r00, r01, r02, tx,
           r10, r11, r12, ty,
           r20, r21, r22, tz,
           0.0, 0.0, 0.0, 1.0 ]

# IMU.T_b_c1 = inverse of cam0.T_cam_imu
# Kalibr gives T_cam_imu (camera←IMU); ORB_SLAM3 wants T_b_c1 (IMU←camera)
# Use the Python snippet below to compute the inverse.
```

**Compute `IMU.T_b_c1` (inverse of `T_cam_imu`):**

```python
import numpy as np

# Paste your cam0.T_cam_imu values here:
T_cam_imu = np.array([
    [ 0.9998,  0.0021, -0.0200,  0.0110],
    [-0.0020,  0.9999,  0.0031,  0.0005],
    [ 0.0200, -0.0030,  0.9998, -0.0183],
    [ 0.0000,  0.0000,  0.0000,  1.0000],
])

T_b_c1 = np.linalg.inv(T_cam_imu)
print(np.array2string(T_b_c1, precision=4, suppress_small=True))
# Paste the output into IMU.T_b_c1 data: [ ... ]
```

#### Update `config/mission.yaml` colour camera intrinsics

The `marker_detector` and `occupancy_grid_builder` sections use the **colour camera** (not IR). Read the factory intrinsics directly from the driver:

```bash
ros2 topic echo /d455/color/camera_info --once
# K[0]=fx  K[4]=fy  K[2]=cx  K[5]=cy
```

Then update both sections in `mission.yaml`:

```yaml
marker_detector:
  ros__parameters:
    fx: <K[0]>
    fy: <K[4]>
    cx: <K[2]>
    cy: <K[5]>

occupancy_grid_builder:
  ros__parameters:
    fx: <K[0]>
    fy: <K[4]>
    cx: <K[2]>
    cy: <K[5]>
```

---

### Phase 4 — Verify the Calibration

#### Check time offset

In `camchain-imucam.yaml`, look for `timeshift_cam_imu`. A value of ±5 ms is normal for the D455 (hardware-synced IMU). If you see > 20 ms, verify `unite_imu_method: linear_interpolation` is set in the RealSense driver and re-record.

#### Bench VIO test

```bash
ros2 launch greenhouse_nav sensors_only.launch.py
ros2 topic echo /camera/camera/imu --once   # should see ~400 Hz gyro, 250 Hz accel
ros2 topic echo /poseimu       # move drone by hand — pose should track smoothly
```

Move the drone 1 m and back. Drift should be < 5 cm over 2 m of hand travel. If drift is larger, recheck that you replaced all four calibration blocks and that `cam0_distortion_model: "radtan"` is set (not `"none"`).

---

### Calibration quick-reference

| Kalibr field | Copy to | Notes |
|---|---|---|
| `cam0.intrinsics` | `d455_vio.yaml` `cam0_intrinsics` | Left IR fx/fy/cx/cy |
| `cam0.distortion_coeffs` | `d455_vio.yaml` `cam0_distortion_coeffs` | k1 k2 p1 p2 |
| `cam0.T_cam_imu` | `d455_vio.yaml` `T_cam0_imu` | Camera←IMU 4×4 |
| `cam1.intrinsics` | `d455_vio.yaml` `cam1_intrinsics` | Right IR |
| `cam1.T_cam_imu` | `d455_vio.yaml` `T_cam1_imu` | |
| `cam0.intrinsics` | `d455_orbslam3.yaml` `Camera.fx/fy/cx/cy` | Same values |
| `cam1.intrinsics` | `d455_orbslam3.yaml` `Camera2.*` | |
| `cam1.T_cn_cnm1` | `d455_orbslam3.yaml` `Stereo.T_c1_c2` | Right-to-left extrinsic |
| `cam0.T_cam_imu` **inverted** | `d455_orbslam3.yaml` `IMU.T_b_c1` | Use `np.linalg.inv()` |
| `/d455/color/camera_info` K | `mission.yaml` `marker_detector` + `occupancy_grid_builder` | Colour camera only |

---

## PX4 Parameters (One-time Setup)

Connect Pixhawk via USB, then run the helper:

```bash
python3 ~/ros2_ws/src/greenhouse_nav/greenhouse_nav/set_px4_params.py
```

Or set manually in QGroundControl → Vehicle Setup → Parameters:

| Parameter | Value | Notes |
|---|---|---|
| `EKF2_EV_CTRL` | 15 | Enable all vision fusion (pos + yaw + vel) |
| `EKF2_EV_DELAY` | 50 | ms — tune with `estimator_innovations` log |
| `EKF2_HGT_REF` | 3 | Height reference = vision |
| `EKF2_GPS_CTRL` | 0 | Disable GPS indoors 	|
| `EKF2_OF_CTRL` | 1 | Enable optical flow |
| `EKF2_RNG_CTRL` | 1 | Enable rangefinder |
| `CP_DIST` | 0.8 | Collision prevention clearance (m) |
| `CP_DELAY` | 0.4 | CP sensor latency (s) |
| `CP_GO_NO_DATA` | 0 | Stop if depth data lost |
| `MPC_POS_MODE` | 0 | Required for CP to work |
| `MPC_XY_VEL_MAX` | 0.8 | Max horizontal speed (m/s) |
| `MPC_LAND_SPEED` | 0.3 | Landing descent speed (m/s) |
| `COM_RCL_EXCEPT` | 4 | No RC-loss failsafe in offboard mode |

**Reboot Pixhawk after setting parameters.**

---

## Running the System

## Source (Tony)


sudo nmcli connection up enP8p1s0
MicroXRCEAgent udp4 -p 8888

source /opt/ros/humble/setup.bash
source ~/realsense_ros_ws/install/setup.bash
source ~/ros2_ws/install/setup.bash


ros2 launch greenhouse_nav greenhouse_flight.launch.py

### Every terminal needs

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

source /opt/ros/humble/setup.bash
source ~/workspace/realsense_ws/install/setup.bash
source ~/ws_openvins/install/setup.bash
source ~/Documents/AgriNav-Drone-localization-in-greenhouse/greenhouse_nav/install/setup.bash
colcon build --packages-select greenhouse_nav --symlink-install
source install/setup.bash

export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

colcon build --packages-select greenhouse_nav
source install/setup.bash


```

### Terminal 1 — uXRCE-DDS Agent (always required)

```bash
MicroXRCEAgent udp4 -p 8888
```

Enable in PX4: QGC → Parameters → `UXRCE_DDS_CFG = 1000` (Ethernet).

### Terminal 2 — Launch the full mission

```bash
# OpenVINS back-end (default)
ros2 launch greenhouse_nav greenhouse_flight.launch.py

# ORB_SLAM3 back-end
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    vio_backend:=orbslam3 \
    orb_vocab:=/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt

# Custom row length / altitude
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    mission_distance:=15.0 takeoff_height:=1.8

# Skip marker init gate (bench testing only — not recommended for flight)
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    require_marker_init:=false
```

### Launch arguments

| Argument | Default | Description |
|---|---|---|
| `vio_backend` | `openvins` | VIO back-end: `openvins` or `orbslam3` |
| `orb_vocab` | `/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt` | Path to ORBvoc.txt |
| `orb_pkg` | `orb_slam3_ros2` | ORB_SLAM3 ROS 2 package name |
| `orb_exe` | `stereo_inertial` | ORB_SLAM3 executable name |
| `mission_distance` | `10.0` | Row length in metres |
| `takeoff_height` | `1.5` | Flight altitude AGL in metres |
| `require_marker_init` | `true` | Hold until ID-0 marker detected before flying |

### Node startup order (handled automatically by launch file)

```
t=0s    D455 camera
t=1s    VIO (OpenVINS or ORB_SLAM3)
t=2s    VIO bridge + occupancy grid + DWA planner + safety + marker detector
t=5s    Mission executor  (OpenVINS)
t=10s   Mission executor  (ORB_SLAM3 — waits for vocabulary load + VI init)
```

---

## Pre-flight Checklist

Work through these steps in order. **Never skip a scenario.**

### Step 0 — Bench Test (props OFF)

```bash
# Terminal 1
MicroXRCEAgent udp4 -p 8888

# Terminal 2
ros2 launch greenhouse_nav sensors_only.launch.py
# OR with ORB_SLAM3:
ros2 launch greenhouse_nav sensors_only.launch.py vio_backend:=orbslam3
```

**Verify each of these before proceeding:**

```bash
# 1. VIO is publishing
ros2 topic echo /poseimu --once                         # OpenVINS
ros2 topic echo /orb_slam3/camera_pose --once           # ORB_SLAM3

# 2. Bridge is converting and forwarding to PX4
ros2 topic echo /fmu/in/vehicle_visual_odometry --once
# Expected: NED position, quaternion, velocity (not all NaN after ~1 s)

# 3. PX4 EKF2 is fusing VIO  ← most important
ros2 topic echo /fmu/out/vehicle_local_position --once
# Expected: xy_valid=true, z_valid=true
# If false: EKF2_EV_CTRL is wrong or uXRCE-DDS is not connected

# 4. Marker detector is working — hold ID-0 marker in front of camera
ros2 topic echo /marker/event
# Expected: {"event": "MARKER_INIT", "marker_id": 0, "distance_m": 1.23}

# 5. View annotated camera feed (optional)
ros2 run rqt_image_view rqt_image_view /marker/debug_image

# 6. Move drone by hand 1 m — position should track
ros2 topic echo /fmu/out/vehicle_local_position --field x
# Drift should be < 5 cm over 2 m of hand movement
```

**Pass criteria:** All 6 checks pass. Do not arm until they do.

---

### Step 1 — Hover Test, 0.5 m, Tethered

First armed flight. Props ON, spotter holding tether.

```bash
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    mission_distance:=0.5 takeoff_height:=0.8
```

Hold the ID-0 marker in front of the camera → drone will start flying.

**Monitor:**
```bash
ros2 topic echo /mission/status        # phase transitions
ros2 topic echo /safety/status         # watchdog
ros2 topic echo /fmu/out/estimator_innovations  # VIO quality
```

**Pass criteria:** Stable hover at 0.8 m ± 0.15 m, lands within 0.4 m of start.

---

### Step 2 — Single Row, No Obstacles

```bash
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    mission_distance:=5.0
```

Place ID-0 at start, ID-10 (landing marker) at 5 m. No U-turn markers needed.

**Pass criteria:** Drone detects start marker, flies 5 m, detects land marker, lands.

---

### Step 3 — Two-Row Mission

Place markers:
```
[ID 0] ──── 10 m ──── [ID 1 or 2]
                       lateral shift 2 m
[ID 10] ─── 10 m ────[start of row 2]
```

```bash
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    mission_distance:=10.0
```

**Pass criteria:** Drone completes the U-turn, flies the second row, detects landing marker, lands.

---

### Step 4 — Obstacles in Row

Add poles or plants inside the rows after Step 3 passes.

```bash
# Watch occupancy grid and planner in RViz2
ros2 topic echo /dwa/status
# Expected near obstacle: RECOVERY or adjusted velocity
```

---

### Step 5 — Full Greenhouse Mission

Multiple rows, realistic obstacle layout, full `mission_distance`.

```bash
ros2 launch greenhouse_nav greenhouse_flight.launch.py \
    mission_distance:=15.0 \
    vio_backend:=orbslam3 \
    orb_vocab:=/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
```

---

## Tuning Guide

### VIO is drifting

1. Re-run Kalibr calibration — wrong intrinsics are the #1 drift source
2. Increase `num_pts` in `d455_vio.yaml` (try 300) for OpenVINS
3. Increase `ORBextractor.nFeatures` in `d455_orbslam3.yaml` (try 1250) for ORB_SLAM3
4. Reduce `cruise_speed` — VIO degrades at high speed
5. Check lighting — needs ≥ 50 lux; avoid harsh shadows on the IR cameras

### ORB_SLAM3 loses tracking in narrow rows

- Ensure IR projector is OFF (`emitter_enabled 0`) — structured light interferes with feature matching
- Increase `ORBextractor.nFeatures` to 1500
- Reduce drone speed so more ORB features are tracked per frame

### Drone oscillates laterally

- Reduce `w_smooth` in `dwa_planner` section of `mission.yaml` (try 1.2)
- Increase `max_accel` slightly (try 0.4)
- Check `MPC_XY_VEL_MAX` ≤ 0.8 m/s

### DWA keeps reporting BLOCKED / RECOVERY

- Obstacle is wider than `hard_stop_dist` allows passage
- Reduce `inflation_radius_m` in steps of 0.05 m
- Verify actual gap between obstacles ≥ 1.3 m
- Check `recovery_scan_dist_m` is at least 2.0 m so sector scanner can find free directions

### U-turn overshoots or undershoots

- Increase `uturn_settle_sec` in `mission.yaml` (try 3.0)
- Reduce `MPC_YAW_RATE_MAX` in PX4 (try 60 deg/s)
- Tune `yaw_tol_deg` — reduce for more precise U-turn (try 5.0°)

### Marker detector fires at wrong distance / false triggers

- Increase `min_consecutive` (try 3) to require more consecutive detections
- Increase `detection_cooldown_sec` (try 3.0) to prevent re-triggering
- Ensure `marker_size_m` matches the physically printed size exactly
- Run `ros2 topic echo /marker/debug_image` in rqt_image_view to see what the detector sees

### Mission executor ignores marker events

- Confirm `/marker/event` is publishing: `ros2 topic echo /marker/event`
- Check the `marker_id_*` parameters in `mission.yaml` match what's printed on the markers
- Verify the correct phase is active — e.g., `MARKER_UTURN_LEFT` only fires during `FORWARD` phase

---

## Safety Rules

1. **Always fly with a human spotter** who has RC controller ready to take manual control
2. **Set `COM_RCL_EXCEPT = 4`** but keep RC link active as emergency backup
3. **Never skip bench test (Step 0)** — if VIO is not fusing, do not arm
4. **Increase `mission_distance` in 2–3 m increments** — never jump straight to 15 m
5. **Safety monitor auto-lands** if VIO is lost > 1 s or altitude exceeded — do not disable
6. **Disable IR projector** before every flight: `ros2 param set /camera/camera depth_module.emitter_enabled 0`
7. **Do not fly if ORB_SLAM3 status shows TRACKING_LOST** at launch — wait for re-init

---

## Hardware Connection Reference

```
Jetson Orin NX
  ├── USB 3.2   → Intel RealSense D455
  ├── Ethernet  → Pixhawk 6X  (uXRCE-DDS UDP, port 8888)
  └── USB (opt) → Pixhawk 6X  (parameter setting / QGC)

Pixhawk 6X
  ├── DroneCAN  → RM3100 Compass
  ├── I2C       → Optical Flow + Distance Sensor
  └── UART      → RC receiver (keep as emergency backup)
```

---

## Quick Reference

```bash
# ── Environment setup ─────────────────────────────────────────────────────── #
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ── Build after code changes ──────────────────────────────────────────────── #
cd ~/ros2_ws
colcon build --packages-select greenhouse_nav --symlink-install
source install/setup.bash

# ── Always start first ────────────────────────────────────────────────────── #
MicroXRCEAgent udp4 -p 8888

# ── Disable D455 IR projector (must do before every VIO session) ──────────── #
ros2 param set /camera/camera depth_module.emitter_enabled 0

# ── Bench test (no flight) ────────────────────────────────────────────────── #
ros2 launch greenhouse_nav sensors_only.launch.py
ros2 launch greenhouse_nav sensors_only.launch.py vio_backend:=orbslam3

# ── Full mission ──────────────────────────────────────────────────────────── #
ros2 launch greenhouse_nav greenhouse_flight.launch.py
ros2 launch greenhouse_nav greenhouse_flight.launch.py vio_backend:=orbslam3 orb_vocab:=/opt/ORB_SLAM3/Vocabulary/ORBvoc.txt
ros2 launch greenhouse_nav greenhouse_flight.launch.py mission_distance:=15.0 takeoff_height:=1.8

# ── Monitoring ────────────────────────────────────────────────────────────── #
ros2 topic echo /mission/status                          # current mission phase
ros2 topic echo /safety/status                           # watchdog status
ros2 topic echo /marker/event                            # marker detections
ros2 topic echo /orb_slam3/status                        # ORB_SLAM3 tracking state
ros2 topic echo /fmu/out/vehicle_local_position_v1       # PX4 local position
ros2 topic echo /fmu/out/estimator_innovations           # EKF2 VIO fusion quality
ros2 run rqt_image_view rqt_image_view /marker/debug_image  # annotated camera feed

# ── PX4 parameter helper ──────────────────────────────────────────────────── #
python3 ~/ros2_ws/src/greenhouse_nav/greenhouse_nav/set_px4_params.py
```

---

## License

MIT License — see LICENSE file.
