#!/bin/bash
# Opens a window showing the YOLO debug image (camera + bounding boxes + labels).
# Run this in a separate terminal AFTER fruit_detector.sh is up.

source ~/ros2_jazzy/install/setup.bash
cd ~/ros2_ws
source install/setup.bash

exec ros2 run yolo_speaker debug_viewer_node
