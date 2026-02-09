cmake_minimum_required(VERSION 3.8)
project(color_blob_centroid)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(color_names REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED)
find_package(color_tools_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

########################## OpenCV ###################################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(include)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  message_filters
  tf2_ros
  tf2
  tf2_geometry_msgs
  std_msgs
  sensor_msgs
  image_transport
  geometry_msgs
  cv_bridge
  color_names
  color_tools_msgs
  std_srvs
)

# Add the processing library
add_library(color_blob_centroid SHARED
  src/color_blob_centroid.cpp
)
target_link_libraries(color_blob_centroid ${OpenCV_LIBS})
ament_target_dependencies(color_blob_centroid ${dependencies})

# Main node executable
add_executable(color_blob_node src/color_blob_node.cpp)
target_link_libraries(color_blob_node color_blob_centroid ${OpenCV_LIBS})
ament_target_dependencies(color_blob_node ${dependencies})

# Client executable
set(client_dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  tf2
  tf2_ros
  std_msgs
  sensor_msgs
  image_transport
  cv_bridge
  color_names
  color_tools_msgs
  OpenCV
)
add_executable(centroid_client src/centroid_client.cpp)
ament_target_dependencies(centroid_client ${client_dependencies})

# Install library, includes, and executables
install(TARGETS
  color_blob_centroid
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS
  color_blob_node
  centroid_client
  DESTINATION lib/${PROJECT_NAME}/
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(color_blob_centroid_test test/color_blob_centroid_test.cpp)
  target_link_libraries(color_blob_centroid_test color_blob_centroid ${OpenCV_LIBS})
  ament_target_dependencies(color_blob_centroid_test ${dependencies})
endif()

ament_export_include_directories(include)
ament_export_libraries(color_blob_processor)
ament_export_dependencies(${dependencies})

ament_package()
