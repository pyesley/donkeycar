#!/bin/bash
# Launch script for PiDog ROS2 node (Camera + Audio)

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "========================================"
echo "PiDog ROS2 Node"
echo "========================================"
echo ""
echo "Topics Published:"
echo "  - pidog/camera/image_raw"
echo "  - pidog/camera/image_raw/compressed"
echo "  - pidog/audio/capture"
echo "  - pidog/audio/status"
echo ""
echo "Topics Subscribed:"
echo "  - pidog/audio/playback"
echo ""
echo "Press Ctrl+C to stop"
echo "========================================"
echo ""

ros2 run pidog_ros pidog_node
