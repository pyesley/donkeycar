cmake_minimum_required(VERSION 3.8)
project(personaplex_bridge)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Find Boost (for Beast WebSocket)
find_package(Boost REQUIRED COMPONENTS system)

# Find OpenSSL
find_package(OpenSSL REQUIRED)

# Find threading
find_package(Threads REQUIRED)

# Define the executable
add_executable(personaplex_bridge_node
  src/main.cpp
  src/personaplex_bridge_node.cpp
)

# Include directories
target_include_directories(personaplex_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Boost_INCLUDE_DIRS}
)

# Link ROS libraries
ament_target_dependencies(personaplex_bridge_node
  rclcpp
  std_msgs
)

# Link Boost, OpenSSL, and threading
target_link_libraries(personaplex_bridge_node
  ${Boost_LIBRARIES}
  OpenSSL::SSL
  OpenSSL::Crypto
  Threads::Threads
)

# Install rules
install(TARGETS
  personaplex_bridge_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
