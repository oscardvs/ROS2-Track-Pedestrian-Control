cmake_minimum_required(VERSION 3.8)
project(opencv_person_detector)

# Enable compiler warnings for better code quality
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(vision_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Add executable
add_executable(opencv_person_detector_node
  src/opencv_person_detector_node.cpp
  src/opencv_person_detector_main.cpp
)


# Specify dependencies for the executable
ament_target_dependencies(opencv_person_detector_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  vision_msgs
)

# Enable testing and linting
install(TARGETS
  opencv_person_detector_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()



