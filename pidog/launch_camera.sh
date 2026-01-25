#!/bin/bash
# Launch script for PiDog camera node

cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

echo "Starting PiDog Camera Node..."
echo "Publishing to: pidog/camera/image_raw"
echo "Press Ctrl+C to stop"
echo ""

ros2 run pidog_camera pidog_camera_node
