cmake_minimum_required(VERSION 3.8)
project(openmanipulator_custom_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 1. Create the library
add_library(my_controller SHARED
  src/my_controller.cpp
)

target_include_directories(my_controller PUBLIC
  include
)

ament_target_dependencies(my_controller
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 2. Install the library
install(
  TARGETS my_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 3. Install the plugin.xml
install(
  FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# 4. Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# =========================================================
# CRITICAL FIX: Manually Register in Ament Index
# This creates the "marker" file that 'ros2 control' looks for.
# =========================================================
ament_index_register_resource("controller_interface")

install(
  FILES plugin.xml # The file content usually doesn't matter for the marker, but we need A file.
  DESTINATION share/ament_index/resource_index/controller_interface
  RENAME openmanipulator_custom_controller # The marker MUST be named after your package
)
# =========================================================

ament_package()