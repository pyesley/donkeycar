cmake_minimum_required(VERSION 3.8)
project(rpi_ros2_cpp_nodes)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Define the main executable that will run all nodes
add_executable(rpi_main_nodes
  src/main.cpp
  src/arduino_ros_bridge_node.cpp
  src/camera_stream_node.cpp
  src/imu_6050_node.cpp
)

# Target include directories
target_include_directories(rpi_main_nodes PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link libraries using ament_target_dependencies
ament_target_dependencies(rpi_main_nodes
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  rcl_interfaces
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(rpi_main_nodes
  ${OpenCV_LIBRARIES}
  pthread
)

# Install rules
install(TARGETS
  rpi_main_nodes
  DESTINATION lib/${PROJECT_NAME}
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Linters
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()