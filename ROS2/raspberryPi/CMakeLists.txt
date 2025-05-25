cmake_minimum_required(VERSION 3.8)
project(ros2_raspberry_pi_nodes_cpp) # Changed project name to be more general

# Find necessary ROS2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED) # For arduino_bridge_node
find_package(nav_msgs REQUIRED)      # For arduino_bridge_node
find_package(rcl_interfaces REQUIRED) # For arduino_bridge_node
find_package(tf2_geometry_msgs REQUIRED) # For arduino_bridge_node
find_package(sensor_msgs REQUIRED)   # For camerastream_node
find_package(cv_bridge REQUIRED)     # For camerastream_node

# Find Boost (for arduino_bridge_node)
find_package(Boost REQUIRED COMPONENTS system thread)

# Find OpenCV (for camerastream_node)
find_package(OpenCV REQUIRED COMPONENTS core imgproc) # imgcodecs might be needed if reading/writing images to disk

# Find GStreamer (for camerastream_node)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP REQUIRED gstreamer-app-1.0)

# Include directories for GStreamer
include_directories(${GSTREAMER_INCLUDE_DIRS} ${GSTREAMER_APP_INCLUDE_DIRS})

# Library for Arduino Bridge
add_library(arduino_ros_bridge_lib arduino_ros_bridge.cpp)
ament_target_dependencies(arduino_ros_bridge_lib
    rclcpp
    geometry_msgs
    nav_msgs
    rcl_interfaces
    tf2_geometry_msgs
    Boost::system
    Boost::thread
)
target_link_libraries(arduino_ros_bridge_lib PUBLIC # PUBLIC allows unified_runner_node to link these too
    Boost::system
    Boost::thread
)
install(TARGETS arduino_ros_bridge_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin # Should not be needed for a library, but common practice
)

# Library for Camera Stream
add_library(camerastream_lib camerastream.cpp)
ament_target_dependencies(camerastream_lib
    rclcpp
    sensor_msgs
    cv_bridge
    OpenCV::opencv_core
    OpenCV::opencv_imgproc
)
target_link_libraries(camerastream_lib PUBLIC # PUBLIC for transitive linking
    OpenCV::opencv_core
    OpenCV::opencv_imgproc
    ${GSTREAMER_LIBRARIES}
    ${GSTREAMER_APP_LIBRARIES}
)
install(TARGETS camerastream_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Library for MPU6050 IMU Publisher
add_library(imu_6050_lib IMU_6050.cpp)
ament_target_dependencies(imu_6050_lib
    rclcpp
    sensor_msgs
)
# No special system libs to link for imu_6050_lib beyond what rclcpp and sensor_msgs bring
install(TARGETS imu_6050_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# Main executable that runs all nodes
add_executable(unified_runner_node main.cpp)
ament_target_dependencies(unified_runner_node
    rclcpp
    arduino_ros_bridge_lib
    camerastream_lib
    imu_6050_lib
)
target_link_libraries(unified_runner_node
    rclcpp # Already covered by ament_target_dependencies but good practice
    arduino_ros_bridge_lib
    camerastream_lib
    imu_6050_lib
)
install(TARGETS
    unified_runner_node
    DESTINATION lib/${PROJECT_NAME} # Standard install location for executables
)

ament_package()
