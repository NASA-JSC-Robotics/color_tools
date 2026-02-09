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

# Optionally build and install the nanobind python bindings
option(BUILD_BINDINGS "Build Python bindings using nanobind" ON)

if(BUILD_BINDINGS)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
  find_package(ament_cmake_python REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_generator_cpp REQUIRED)

  include(FetchContent)
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v2.11.0
  )
  FetchContent_MakeAvailable(nanobind)

  nanobind_add_module(
    bindings
    STABLE_ABI
    NB_STATIC
    src/bindings.cpp
  )
  target_include_directories(bindings PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${rosidl_generator_cpp_INCLUDE_DIRS}
  )
  set_property(TARGET bindings PROPERTY CXX_STANDARD 20)
  target_link_libraries(bindings PRIVATE
    color_blob_centroid
    ${sensor_msgs_TARGETS}
    ${color_tools_msgs_TARGETS}
    ${rosidl_generator_cpp_TARGETS}
  )
  ament_python_install_package(${PROJECT_NAME})
  ament_get_python_install_dir(PYTHON_INSTALL_DIR)
  install(TARGETS bindings
    LIBRARY DESTINATION "${PYTHON_INSTALL_DIR}/color_blob_centroid"
  )
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/__init__.py
    DESTINATION "${PYTHON_INSTALL_DIR}/color_blob_centroid"
  )
endif()

# Include tests
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(color_blob_centroid_test test/color_blob_centroid_test.cpp)
  target_link_libraries(color_blob_centroid_test color_blob_centroid ${OpenCV_LIBS})
  ament_target_dependencies(color_blob_centroid_test ${dependencies})


  if(BUILD_BINDINGS)
    find_package(ament_cmake_pytest REQUIRED)
    ament_get_python_install_dir(PYTHON_INSTALL_DIR)
    ament_add_pytest_test(test_color_blob_centroid_py test/test_color_blob_centroid.py
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../
      APPEND_ENV PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${PYTHON_INSTALL_DIR}
    )
  endif()
endif()

ament_export_include_directories(include)
ament_export_libraries(color_blob_centroid)
ament_export_dependencies(${dependencies})

ament_package()
