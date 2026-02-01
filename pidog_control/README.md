# PiDog Control Package

ROS2 Python package for controlling PiDog with a desktop dashboard.

## Components

### Motion Node (runs on PiDog)

Receives motion commands and controls the robot's legs.

```bash
ros2 run pidog_control motion_node
```

### Dashboard (runs on Desktop)

GUI application for viewing camera/IMU and controlling PiDog.

```bash
ros2 run pidog_control dashboard
```

## Topics

### Subscribed by Motion Node

| Topic | Type | Description |
|-------|------|-------------|
| `/pidog/cmd_vel` | geometry_msgs/Twist | Velocity commands (linear.x, angular.z) |
| `/pidog/cmd` | std_msgs/String | Discrete commands |

### Published by Motion Node

| Topic | Type | Description |
|-------|------|-------------|
| `/pidog/motion/status` | std_msgs/String | Current motion state |

### Subscribed by Dashboard

| Topic | Type | Description |
|-------|------|-------------|
| `/pidog/camera/image_raw/compressed` | sensor_msgs/CompressedImage | Camera feed |
| `/pidog/imu` | sensor_msgs/Imu | IMU data |
| `/pidog/motion/status` | std_msgs/String | Motion status |

### Published by Dashboard

| Topic | Type | Description |
|-------|------|-------------|
| `/pidog/cmd_vel` | geometry_msgs/Twist | Velocity commands |
| `/pidog/cmd` | std_msgs/String | Discrete commands |

## Keyboard Controls (Dashboard)

### Motion
| Key | Action |
|-----|--------|
| W | Walk forward |
| S | Walk backward |
| A | Turn left |
| D | Turn right |
| Q | Forward + turn left |
| E | Forward + turn right |
| Space | Stop |

### Commands
| Key | Action |
|-----|--------|
| 1 | Stand |
| 2 | Sit |
| 3 | Rest |
| 4 | Wave |
| 5 | Stretch |
| 6 | Shake |
| Esc | Quit |

## Available Commands

| Command | Description |
|---------|-------------|
| `stand` | Stand in neutral position |
| `sit` | Sit down |
| `rest` | Go to rest/sleep position |
| `wave` | Wave with front paw |
| `stretch` | Do a stretch animation |
| `shake` | Shake body |
| `pushup` | Do pushups |
| `stop` | Stop all motion |

## Dependencies

### PiDog (motion_node)
- ROS2 Jazzy
- pidog_fusion module (in ~/pidog/claude)

### Desktop (dashboard)
- ROS2 Jazzy
- PySide6: `pip install PySide6`
- pyqtgraph: `pip install pyqtgraph`
- OpenCV: `pip install opencv-python`

## Installation

```bash
# On both PiDog and Desktop
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pidog_control
source install/setup.bash
```

## Running

### On PiDog

```bash
# Terminal 1: Main sensor node
ros2 run pidog_ros pidog_node

# Terminal 2: Motion controller
ros2 run pidog_control motion_node
```

### On Desktop

```bash
# Ensure same ROS_DOMAIN_ID as PiDog
export ROS_DOMAIN_ID=42

ros2 run pidog_control dashboard
```

## Architecture

```
Desktop                              PiDog (Pi 5)
┌─────────────────────┐              ┌─────────────────────┐
│  pidog_dashboard    │              │  pidog_ros (C++)    │
│                     │              │  - CameraNode       │
│  - Camera display   │◄── WiFi ────│  - AudioNode        │
│  - IMU plots        │    ROS2     │  - IMUNode          │
│  - Keyboard control │    DDS      │                     │
│                     │              ├─────────────────────┤
│  cmd_vel ──────────►│              │  motion_node (Py)   │
│  cmd ──────────────►│              │  - Receives cmds    │
│                     │──── WiFi ──►│  - Controls legs    │
└─────────────────────┘              └─────────────────────┘
```
