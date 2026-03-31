# Autonomous Turret Robot — ROS2

An autonomous turret system mounted on an omnidirectional car. Uses **YOLOv8** computer vision to detect, track, and fire at a target in real time. Built entirely on **ROS2 Humble** with a live web dashboard for monitoring and control.

---

## Demo

| Feature | Preview |
|---------|---------|
| YOLOv8 detection with bounding box | Green box tracks target in live feed |
| PID lock-on | Turret aims and holds on target |
| Web dashboard | Full telemetry + steering at `localhost:5000` |
| Instant fire | Fires within 0.6s of locking |

---

## System Architecture

```
┌─────────────┐     /image_raw      ┌─────────────────┐     /detections     ┌──────────────┐
│ camera_node │ ──────────────────► │ detection_node  │ ──────────────────► │ tracker_node │
│  (30 Hz)    │                     │  (YOLOv8s)      │                     │              │
└─────────────┘                     └─────────────────┘                     └──────┬───────┘
                                                                                    │
                                                              /target_error  ────── ┤
                                                              /target_detected ───── ┤
                                                                                    │
                                                                            ┌───────▼───────┐     /imu_compensation
                                                                            │   pid_node    │ ◄─────────────────────
                                                                            │  (P control)  │          imu_node
                                                                            └───────┬───────┘
                                                                                    │
                                                         /servo_cmd ────────────────┤
                                                         /locked    ────────────────┤
                                                                            ┌───────┴──────┐    ┌───────────┐
                                                                            │  sc15_node   │    │ fire_node │
                                                                            │ (pan/tilt)   │    │ (BLDC+    │
                                                                            └──────────────┘    │  trigger) │
                                                                                                └───────────┘
┌───────────────┐     /cmd_vel
│ drive_node    │ ◄────────────── dashboard_node  ──► http://localhost:5000
│ (omni wheels) │
└───────────────┘
```

---

## Packages

| Package | Nodes | Description |
|---------|-------|-------------|
| `turret_camera` | `camera_node` | Captures frames from USB/Pi camera at 30 Hz |
| `turret_vision` | `detection_node`, `tracker_node` | YOLOv8 object detection + pixel error calculation |
| `turret_control` | `pid_node`, `sc15_node`, `fire_node`, `imu_node`, `dashboard_node` | Aiming, firing, IMU compensation, web dashboard |
| `turret_drive` | `drive_node` | 4-wheel omnidirectional car control |

---

## ROS2 Topic Map

| Topic | Type | Publisher | Subscribers |
|-------|------|-----------|-------------|
| `/image_raw` | `sensor_msgs/Image` | camera_node | detection_node, dashboard_node |
| `/detections` | `std_msgs/String` (JSON) | detection_node | tracker_node, dashboard_node |
| `/target_error` | `std_msgs/Float32MultiArray` | tracker_node | pid_node, dashboard_node |
| `/target_detected` | `std_msgs/Bool` | tracker_node | fire_node, dashboard_node |
| `/servo_cmd` | `std_msgs/Float32MultiArray` | pid_node | sc15_node, dashboard_node |
| `/locked` | `std_msgs/Bool` | pid_node | fire_node, dashboard_node |
| `/turret_status` | `std_msgs/String` | pid_node | dashboard_node |
| `/imu_compensation` | `std_msgs/Float32MultiArray` | imu_node | pid_node |
| `/cmd_vel` | `geometry_msgs/Twist` | dashboard_node | drive_node |
| `/set_target` | `std_msgs/String` | dashboard_node | detection_node, tracker_node |
| `/fire_enable` | `std_msgs/Bool` | dashboard_node | fire_node |

---

## Hardware

| Component | Details |
|-----------|---------|
| **Computer** | Raspberry Pi 5 (4GB/8GB) |
| **Camera** | Pi Camera Module 3 / USB webcam |
| **Pan servo** | Waveshare SC15 serial bus servo (ID 1) |
| **Tilt servo** | Waveshare SC15 serial bus servo (ID 2) |
| **Trigger** | Metal gear servo (standard PWM) |
| **Shooter** | 2× BLDC motors (flywheel) |
| **Drive** | 4× yellow TT DC motors + L298N driver |
| **IMU** | MPU-6050 (I2C 0x68) |
| **Chassis** | 4× omni wheels |
| **Power** | 4S LiPo 11.1V + 5V BEC for Pi |

> **Note:** All GPIO/hardware code is included but commented out. The project runs fully in **simulation mode** on a laptop — no hardware required to test.

---

## Installation

### Requirements
- Ubuntu 22.04 LTS
- ROS2 Humble
- Python 3.10+

### 1. Install ROS2 Humble

```bash
sudo apt install software-properties-common -y && sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list
sudo apt update && sudo apt install ros-humble-desktop python3-colcon-common-extensions -y
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

### 2. Clone and build

```bash
git clone https://github.com/YOUR_USERNAME/turret-robot.git ~/turret_ws
cd ~/turret_ws
```

### 3. Install dependencies

```bash
sudo apt install ros-humble-cv-bridge ros-humble-teleop-twist-keyboard -y
pip3 install opencv-python ultralytics flask flask-socketio "numpy<2"
```

### 4. Build

```bash
cd ~/turret_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Usage

### Start everything

```bash
ros2 launch turret_control turret.launch.py
```

Open **http://localhost:5000** in your browser.

### Optional: keyboard car control

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
`i` = forward · `,` = back · `j` = left · `l` = right · `k` = stop

---

## Web Dashboard

| Panel | Description |
|-------|-------------|
| Camera feed | Live stream with green detection box + red crosshair |
| Status bar | Target locked/searching, fire ready/fired, shot counter |
| Error graph | Live 50-point chart of tracking error X and Y |
| Car steering | D-pad buttons + speed slider (also W/A/S/D keyboard) |
| Target selector | Switch between Bottle / Cup / Person / Ball / Phone live |
| Fire safety | Toggle to enable/disable firing |
| Event log | Color-coded LOCKED, FIRE, SAFETY events |

---

## Key Parameters

| Node | Parameter | Default | Effect |
|------|-----------|---------|--------|
| `detection_node` | YOLOv8 model | `yolov8s.pt` | Swap for `yolov8n` (faster) or `yolov8m` (more accurate) |
| `detection_node` | Confidence threshold | `0.25` | Lower = more detections, more false positives |
| `pid_node` | `kp` | `0.003` | Higher = faster tracking, may overshoot |
| `pid_node` | `LOCK_THRESHOLD` | `50 px` | Smaller = must be more centered before firing |
| `pid_node` | `LOCK_HOLD_TIME` | `0.3 s` | How long target must stay centered before firing |
| `fire_node` | `COOLDOWN_TIME` | `2.0 s` | Minimum time between shots |

---

## Enabling Hardware (Raspberry Pi)

All hardware code is commented out with `# WITH HARDWARE:` markers.

| Node | What to uncomment | Lines |
|------|-------------------|-------|
| `camera_node` | `VideoCapture('/dev/video0')` | 1 |
| `sc15_node` | Serial UART to SC15 servos | ~5 |
| `fire_node` | GPIO for BLDC + solenoid trigger | ~8 |
| `drive_node` | GPIO for L298N motor driver | ~8 |
| `imu_node` | I2C for MPU-6050 gyroscope | ~6 |

---

## Omni Wheel Kinematics

```
FL = x - y - z        FR = x + y + z
RL = x + y - z        RR = x - y + z

x = linear.x  (forward/backward)
y = linear.y  (strafe)
z = angular.z (rotation)
```

---

## Project Status

- [x] Camera streaming node
- [x] YOLOv8 object detection
- [x] Pixel error tracker
- [x] PID servo controller
- [x] SC15 serial servo driver (simulation)
- [x] Fire sequence with BLDC spinup
- [x] IMU compensation (simulation)
- [x] 4-wheel omni drive (simulation)
- [x] Live web dashboard
- [x] Fire safety toggle
- [x] Dynamic target switching
- [x] Shot counter + live error graph
- [ ] Raspberry Pi hardware integration
- [ ] Real SC15 servo control
- [ ] Real motor GPIO control
- [ ] Real MPU-6050 IMU

---

## License

MIT License — free to use, modify, and build on.
