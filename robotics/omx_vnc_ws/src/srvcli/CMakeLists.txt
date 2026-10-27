cmake_minimum_required(VERSION 3.8)
project(srvcli)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate service interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Direction.srv"
  "srv/SetPose.srv"
  "srv/PerformTask.srv"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()