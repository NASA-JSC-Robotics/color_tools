cmake_minimum_required(VERSION 3.5)
project(dex_ivr_interfaces)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(service_files
  "srv/BlobDimensions.srv"
  "srv/BlobCentroid.srv"
)

set(action_files
  "action/FindWipe.action"
  "action/GuiRun.action"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${service_files}
  ${action_files}
  DEPENDENCIES
    std_msgs
    geometry_msgs
    sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
endif()

ament_package()
