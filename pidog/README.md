# PiDog Camera ROS2 Package

Camera streaming package for Sunfounder PiDog robot running ROS2 Jazzy on Raspberry Pi 5.

## Building

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select pidog_camera
source install/setup.bash
```

## Running the Camera Node

```bash
source ~/ros2_ws/install/setup.bash
ros2 run pidog_camera pidog_camera_node
```

## Topics

- **`pidog/camera/image_raw`** - Raw uncompressed camera images (sensor_msgs/Image)
  - Frame ID: `pidog_camera_frame`
  - Default resolution: 640x480 @ 30 FPS
  - Format: BGR8
  - **Use for:** Local processing on PiDog

- **`pidog/camera/image_raw/compressed`** - JPEG compressed images (sensor_msgs/CompressedImage)
  - Same resolution and frame rate
  - Format: JPEG (default quality: 80)
  - **Use for:** Viewing on laptop over WiFi (10-20x smaller!)

## Parameters

You can override parameters when launching:

```bash
ros2 run pidog_camera pidog_camera_node --ros-args \
  -p img_width:=1920 \
  -p img_height:=1080 \
  -p framerate:=15
```

## Viewing Camera Stream on Laptop

### Method 1: Using rqt_image_view (GUI)

```bash
# On your laptop (ensure ROS2 Jazzy is installed)
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```

Then select `/pidog/camera/image_raw` from the dropdown.

### Method 2: Using command line

```bash
# Check if topic is publishing
ros2 topic list | grep pidog

# Check topic info
ros2 topic info /pidog/camera/image_raw

# Echo topic (text output)
ros2 topic echo /pidog/camera/image_raw --no-arr

# Monitor publish rate
ros2 topic hz /pidog/camera/image_raw
```

### Method 3: Using image_view

```bash
# On your laptop
source /opt/ros/jazzy/setup.bash
ros2 run image_view image_view --ros-args --remap image:=/pidog/camera/image_raw
```

## Network Configuration

Ensure both your PiDog (Raspberry Pi 5) and laptop are on the same network and can discover each other via ROS2 DDS.

### Check Network Connectivity

```bash
# On PiDog, check what's being published
ros2 topic list

# On laptop, check if you can see PiDog's topics
ros2 topic list | grep pidog
```

If topics aren't visible across machines, check:
1. Both machines are on same WiFi network
2. Firewall settings allow multicast traffic
3. ROS_DOMAIN_ID is same on both machines (default is 0)

### Set ROS_DOMAIN_ID (if needed)

```bash
# Add to ~/.bashrc on both machines
export ROS_DOMAIN_ID=42
```

## Files

- `src/main.cpp` - Main entry point, initializes camera node
- `src/camera_stream_node.cpp` - Camera streaming implementation using GStreamer
- `include/pidog_camera/camera_stream_node.hpp` - Header file
- `CMakeLists.txt` - Build configuration
- `package.xml` - Package metadata and dependencies
