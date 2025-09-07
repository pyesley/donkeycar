cmake_minimum_required(VERSION 3.8)
project(rpi_ros2_cpp_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Define the executable
add_executable(rpi_main_nodes
  src/main.cpp
  src/arduino_ros_bridge_node.cpp
  src/camera_stream_node.cpp
  src/imu_bmi088_node.cpp
  src/rplidar_node.cpp
)

# Tell the compiler to look in the local "include" folder
target_include_directories(rpi_main_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS libraries
ament_target_dependencies(rpi_main_nodes
  rclcpp
  std_msgs
  std_srvs
  sensor_msgs
  nav_msgs
  cv_bridge
  image_transport
)

# Link system libraries
target_link_libraries(rpi_main_nodes
  ${OpenCV_LIBRARIES}
)

# Install rules
install(TARGETS
  rpi_main_nodes
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

ament_package()