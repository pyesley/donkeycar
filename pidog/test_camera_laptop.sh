#!/bin/bash
# Test script to check PiDog camera from laptop
# Run this on your LAPTOP (not PiDog)

echo "=== Testing PiDog Camera Stream ==="
echo ""

echo "1. Checking if topic exists..."
ros2 topic list | grep pidog

echo ""
echo "2. Checking topic info..."
ros2 topic info /pidog/camera/image_raw -v

echo ""
echo "3. Testing with BEST_EFFORT QoS (should match camera)..."
echo "This should show ~18-30 Hz"
timeout 10 ros2 topic hz /pidog/camera/image_raw --qos-reliability best_effort

echo ""
echo "=== Test Complete ==="
echo ""
echo "To view camera with correct QoS:"
echo "ros2 run rqt_image_view rqt_image_view --qos-reliability best_effort"
