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

  add_library(${PROJECT_NAME} src/ColorBlobCentroid.cpp)

  set(centroid_dependencies
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
  OpenCV)

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
  OpenCV)

  ament_target_dependencies(${PROJECT_NAME} ${centroid_dependencies})

  add_executable(ColorBlobCentroid src/ColorBlobCentroid.cpp)
  ament_target_dependencies(ColorBlobCentroid ${centroid_dependencies})

  add_executable(CentroidClient src/CentroidClient.cpp)
  ament_target_dependencies(CentroidClient ${client_dependencies})

  install(TARGETS
    ${PROJECT_NAME}
    ColorBlobCentroid
    CentroidClient
    DESTINATION lib/${PROJECT_NAME}/
    )

  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/)

  if(BUILD_TESTING)
  endif()

  ament_package()
