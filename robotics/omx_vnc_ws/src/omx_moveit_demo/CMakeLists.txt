cmake_minimum_required(VERSION 3.8)
project(omx_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Create the executable
add_executable(simple_move src/simple_move.cpp)
ament_target_dependencies(simple_move rclcpp moveit_ros_planning_interface)

# Install the executable so ros2 run can find it
install(TARGETS simple_move
  DESTINATION lib/${PROJECT_NAME})

ament_package()