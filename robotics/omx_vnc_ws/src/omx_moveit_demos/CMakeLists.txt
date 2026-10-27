cmake_minimum_required(VERSION 3.8)
project(omx_moveit_demos)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(ex11_planning_scene_attach src/ex11_planning_scene_attach.cpp)
ament_target_dependencies(ex11_planning_scene_attach
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  shape_msgs
)

install(TARGETS ex11_planning_scene_attach
  DESTINATION lib/${PROJECT_NAME})

# --- ADD THIS BLOCK ---
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# ----------------------

ament_package()