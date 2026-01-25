cmake_minimum_required(VERSION 3.8)
project(pidog_camera)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find necessary ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

# Define the executable
add_executable(pidog_camera_node
  src/main.cpp
  src/camera_stream_node.cpp
)

# Include directories
target_include_directories(pidog_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Link ROS libraries
ament_target_dependencies(pidog_camera_node
  rclcpp
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Link OpenCV libraries
target_link_libraries(pidog_camera_node
  ${OpenCV_LIBRARIES}
)

# Install rules
install(TARGETS
  pidog_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
