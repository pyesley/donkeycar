# PiDog ROS2 Package

Unified ROS2 package for Sunfounder PiDog robot running on Raspberry Pi 5 with Fusion HAT+.

## Features

- **Camera Streaming**: GStreamer-based camera capture with compressed output for WiFi viewing
- **Audio Streaming**: Bidirectional real-time audio (24kHz, 16-bit mono) for voice interaction
- **Future**: Servo control, IMU data

## Building

```bash
# Install ALSA development library (for audio)
sudo apt install libasound2-dev

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pidog_ros
source install/setup.bash
```

## Running

### Quick Start
```bash
./launch_pidog.sh
```

### Manual
```bash
source ~/ros2_ws/install/setup.bash
ros2 run pidog_ros pidog_node
```

### With Launch File
```bash
ros2 launch pidog_ros pidog.launch.py
```

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `pidog/camera/image_raw` | `sensor_msgs/Image` | Uncompressed BGR8 images (640x480 @ 30fps) |
| `pidog/camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | JPEG compressed images |
| `pidog/audio/capture` | `std_msgs/UInt8MultiArray` | Microphone audio (24kHz, 16-bit, mono, 20ms chunks) |
| `pidog/audio/status` | `std_msgs/String` | Audio node status/diagnostics |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `pidog/audio/playback` | `std_msgs/UInt8MultiArray` | Speaker audio (24kHz, 16-bit, mono) |

## Parameters

### Camera Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera.width` | 640 | Image width |
| `camera.height` | 480 | Image height |
| `camera.framerate` | 30 | Frames per second |
| `camera.frame_id` | pidog_camera_frame | TF frame ID |
| `camera.jpeg_quality` | 80 | JPEG compression quality (0-100) |

### Audio Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `audio.capture_device` | plughw:2,0 | ALSA capture device |
| `audio.playback_device` | plughw:2,0 | ALSA playback device |
| `audio.enable_capture` | true | Enable microphone capture |
| `audio.enable_playback` | true | Enable speaker playback |
| `audio.capture_volume` | 80 | Capture volume (0-100) |
| `audio.playback_volume` | 80 | Playback volume (0-100) |

## Audio Format

The audio node uses a format compatible with NVIDIA PersonaPlex:

- **Sample Rate**: 24000 Hz
- **Channels**: 1 (mono)
- **Bits per Sample**: 16 (signed little-endian)
- **Chunk Size**: 20ms (480 frames, 960 bytes)

## Viewing on Desktop

### Camera
```bash
# On your laptop
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
# Select /pidog/camera/image_raw
```

### Audio
```bash
# Monitor audio capture
ros2 topic hz /pidog/audio/capture

# View status
ros2 topic echo /pidog/audio/status
```

## Network Setup

Ensure both PiDog and your desktop are on the same network:

```bash
# Check topics are visible
ros2 topic list | grep pidog

# Set ROS_DOMAIN_ID if needed (same on both machines)
export ROS_DOMAIN_ID=42
```

## Architecture

```
┌─────────────────────────────────────────────────────┐
│                 pidog_node (main.cpp)               │
│                                                     │
│  ┌─────────────────┐    ┌─────────────────┐        │
│  │   CameraNode    │    │   AudioNode     │        │
│  │  (GStreamer)    │    │   (ALSA)        │        │
│  └────────┬────────┘    └────────┬────────┘        │
│           │                      │                  │
│           ▼                      ▼                  │
│  ┌─────────────────┐    ┌─────────────────┐        │
│  │ image_raw       │    │ audio/capture   │        │
│  │ image_raw/comp  │    │ audio/playback  │        │
│  └─────────────────┘    └─────────────────┘        │
│                                                     │
│         MultiThreadedExecutor                       │
└─────────────────────────────────────────────────────┘
```

## Files

```
pidog_ros/
├── CMakeLists.txt
├── package.xml
├── include/pidog_ros/
│   ├── camera_node.hpp
│   └── audio_node.hpp
├── src/
│   ├── main.cpp
│   ├── camera_node.cpp
│   └── audio_node.cpp
├── launch/
│   └── pidog.launch.py
├── launch_pidog.sh
└── README.md
```

## Troubleshooting

### Camera not working
```bash
# Check if camera is detected
libcamera-hello --list-cameras

# Test GStreamer pipeline
gst-launch-1.0 libcamerasrc ! videoconvert ! autovideosink
```

### Audio not working
```bash
# List ALSA devices
arecord -l
aplay -l

# Test recording
arecord -D plughw:2,0 -f S16_LE -r 24000 -c 1 -d 5 test.wav

# Test playback
aplay -D plughw:2,0 test.wav
```

### ALSA device busy
```bash
# Check what's using the device
fuser -v /dev/snd/*

# Kill processes if needed
sudo fuser -k /dev/snd/*
```
