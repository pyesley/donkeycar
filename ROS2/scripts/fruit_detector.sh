#!/bin/bash
# Fruit Detector: brings up camera + IMU + lidar + arduino bridge,
# YOLOv8 object detection, and the espeak announce node.
# Ctrl+C to stop everything cleanly.

source ~/ros2_jazzy/install/setup.bash
cd ~/ros2_ws
source install/setup.bash

PIDS=()

cleanup() {
    echo
    echo "[fruit_detector] Shutting down..."
    for pid in "${PIDS[@]}"; do
        kill -TERM -- -"$pid" 2>/dev/null
    done
    sleep 2
    for pid in "${PIDS[@]}"; do
        kill -KILL -- -"$pid" 2>/dev/null
    done
    exit 0
}
trap cleanup INT TERM

echo "[fruit_detector] Starting rpi_main_nodes (camera + IMU + lidar + arduino)..."
setsid ros2 run rpi_ros2_cpp_nodes rpi_main_nodes \
    --ros-args -p use_yolo:=true &
PIDS+=($!)

# Camera needs a moment to bind before yolo subscribes
sleep 3

echo "[fruit_detector] Starting YOLOv8 (CPU, BEST_EFFORT image QoS)..."
setsid ros2 launch yolo_bringup yolov8.launch.py \
    model:=yolov8n.pt \
    input_image_topic:=/camera/image_raw \
    image_reliability:=2 \
    device:=cpu &
PIDS+=($!)

# Give yolo time to load the model before announce subscribes
sleep 6

echo "[fruit_detector] Starting announce_node (says apple/orange/peach)..."
setsid ros2 run yolo_speaker announce_node &
PIDS+=($!)

echo "[fruit_detector] Running. Press Ctrl+C to stop."
wait
