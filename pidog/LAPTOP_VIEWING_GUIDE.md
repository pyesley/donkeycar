# Viewing PiDog Camera Stream on Your Laptop

This guide shows you how to view the PiDog camera stream from your laptop.

## Prerequisites

Your laptop needs:
- ROS2 Jazzy installed
- Same WiFi network as PiDog (192.168.86.x)
- Same ROS_DOMAIN_ID (default is 0)

## Quick Test - Command Line

```bash
# Check if you can see PiDog's topics
ros2 topic list | grep pidog

# You should see:
# /pidog/camera/image_raw              (uncompressed)
# /pidog/camera/image_raw/compressed   (JPEG - USE THIS!)

# Monitor frame rate - USE COMPRESSED for WiFi viewing!
ros2 topic hz /pidog/camera/image_raw/compressed

# Check topic info
ros2 topic info /pidog/camera/image_raw/compressed

# View image data (text)
ros2 topic echo /pidog/camera/image_raw/compressed --no-arr
```

## Method 1: Using rqt_image_view (Recommended)

This provides a GUI window to view the camera stream.

### Install (if needed)
```bash
sudo apt install ros-jazzy-rqt-image-view
```

### Run
```bash
source /opt/ros/jazzy/setup.bash
ros2 run rqt_image_view rqt_image_view
```

**In the GUI:**
1. Click the dropdown menu at the top
2. Select `/pidog/camera/image_raw/compressed` (⚠️ USE COMPRESSED!)
3. Camera stream should appear at ~18-30 FPS!

**Note:** The compressed topic uses JPEG, reducing bandwidth by 10-20x compared to raw images.

## Method 2: Using image_view

This is a lightweight viewer.

### Install (if needed)
```bash
sudo apt install ros-jazzy-image-view
```

### Run (for compressed - RECOMMENDED)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run image_view image_view --ros-args \
  --remap image:=/pidog/camera/image_raw \
  --remap image/compressed:=/pidog/camera/image_raw/compressed \
  -p image_transport:=compressed
```

Or for uncompressed (slow over WiFi):
```bash
ros2 run image_view image_view --ros-args --remap image:=/pidog/camera/image_raw
```

## Method 3: Using RViz2

RViz2 is the full 3D visualization tool for ROS2.

### Run
```bash
source /opt/ros/jazzy/setup.bash
rviz2
```

**In RViz2:**
1. Click "Add" button (bottom left)
2. Select "By topic" tab
3. Find `/pidog/camera/image_raw`
4. Select "Image" and click OK
5. Adjust the "Image" display settings if needed

## Troubleshooting

### Can't see /pidog/camera/image_raw topic

**Check network connectivity:**
```bash
# On PiDog
hostname -I
# Note the IP address (should be 192.168.86.50)

# On laptop
ping 192.168.86.50
```

**Check ROS_DOMAIN_ID:**
```bash
# On both PiDog and laptop
echo $ROS_DOMAIN_ID
# Should be same (usually 0 or empty)
```

**If different networks or ROS domain issues, set explicitly:**
```bash
# Add to ~/.bashrc on BOTH machines
export ROS_DOMAIN_ID=42

# Then restart terminals
```

**Check multicast:**
ROS2 uses DDS multicast for discovery. Some networks block multicast. Try:
```bash
# On laptop, can you see other ROS2 nodes?
ros2 node list

# Can you see PiDog node?
ros2 node list | grep pidog
```

### Topic exists but no image shows

**Check if camera is publishing:**
```bash
# Should show ~18-20 Hz
ros2 topic hz /pidog/camera/image_raw
```

**Check image size:**
```bash
ros2 topic echo /pidog/camera/image_raw --no-arr
# Look for: height: 480, width: 640
```

### Low framerate or choppy video

**Check network bandwidth:**
- 640x480 @ 20 FPS ≈ 18 Mbps
- Use 5GHz WiFi if possible
- Reduce camera resolution if needed (edit node parameters)

**On PiDog, adjust resolution:**
```bash
ros2 run pidog_camera pidog_camera_node --ros-args \
  -p img_width:=320 \
  -p img_height:=240 \
  -p framerate:=15
```

## Camera Specifications

### Uncompressed Stream (local use)
- **Topic:** `/pidog/camera/image_raw`
- **Message Type:** `sensor_msgs/msg/Image`
- **Format:** BGR8
- **Size:** ~900 KB per frame

### Compressed Stream (WiFi viewing - RECOMMENDED)
- **Topic:** `/pidog/camera/image_raw/compressed`
- **Message Type:** `sensor_msgs/msg/CompressedImage`
- **Format:** JPEG (quality: 80)
- **Size:** ~40-90 KB per frame (10-20x smaller!)

### Common
- **Resolution:** 640x480 (default)
- **Frame Rate:** ~18-30 FPS (actual may vary)
- **Frame ID:** `pidog_camera_frame`

## Next Steps

Once you can view the camera, you can:
1. Record the camera stream: `ros2 bag record /pidog/camera/image_raw`
2. Process images with OpenCV
3. Use for VLA (Vision-Language-Action) model input
4. Integrate with navigation and control nodes

## PiDog IP Address

Your PiDog is at: **192.168.86.50**

You can SSH in: `ssh mdyesley@192.168.86.50`
