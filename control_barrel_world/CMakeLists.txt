cmake_minimum_required(VERSION 3.8)
project(control_barrel_world)

# Enable compiler warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(control_barrel_world_node
  src/control_barrel_world_node.cpp
  src/control_barrel_world_main.cpp
)

# Specify dependencies for the executable
ament_target_dependencies(control_barrel_world_node
  rclcpp
  geometry_msgs
  vision_msgs
)

# Install the executable to the proper directory
install(TARGETS
  control_barrel_world_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install the launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Enable testing and linting
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

