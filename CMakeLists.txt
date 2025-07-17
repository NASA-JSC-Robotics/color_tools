cmake_minimum_required(VERSION 3.5)
project(color_names VERSION 0.1.0 LANGUAGES CXX)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

########################## OpenCV #########################################

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#############################################################################

include_directories(include)

add_library(${PROJECT_NAME} src/ColorNames.cpp
                            src/ColorNameLut.cpp
)

add_executable(version test/version.cpp)
target_link_libraries(version ${OpenCV_LIBS})

add_executable(color_image test/color_image.cpp)
target_link_libraries(color_image ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(color_mask test/color_mask.cpp)
target_link_libraries(color_mask ${PROJECT_NAME} ${OpenCV_LIBS})

add_executable(reorder_lut test/reorder_lut.cpp)
target_link_libraries(reorder_lut ${PROJECT_NAME} ${OpenCV_LIBS})

list(APPEND PACKAGE_LIBRARIES ${PROJECT_NAME})

# Mark executables and/or libraries for installation
install(TARGETS ${PACKAGE_LIBRARIES} EXPORT ${PROJECT_NAME}-targets DESTINATION lib)
install(TARGETS version color_image color_mask
       EXPORT ${PROJECT_NAME}-targets DESTINATION bin)
install(EXPORT ${PROJECT_NAME}-targets DESTINATION lib/cmake/${PROJECT_NAME})

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp" PATTERN "*.hxx"
  PATTERN ".svn" EXCLUDE
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})


# Create cmake config files
# See doc: https://cmake.org/cmake/help/latest/module/CMakePackageConfigHelpers.html#example-generating-package-files
include(CMakePackageConfigHelpers)
configure_package_config_file(${CMAKE_CURRENT_LIST_DIR}/cmake/${PROJECT_NAME}-config.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake
  INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
  NO_CHECK_REQUIRED_COMPONENTS_MACRO)

write_basic_package_version_file(${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake
  VERSION ${PROJECT_VERSION} COMPATIBILITY ExactVersion)

install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config-version.cmake"
  DESTINATION lib/cmake/${PROJECT_NAME})

export(EXPORT ${PROJECT_NAME}-targets FILE ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-targets.cmake)


  # Create VERSION File
# add_custom_command(OUTPUT "VERSION" WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} COMMAND ${CMAKE_COMMAND} -DPROJECT_NAME=${PROJECT_NAME} -P version.cmake )
# add_custom_target(WRITE_VERSION ALL DEPENDS "VERSION" )
# install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/VERSION" DESTINATION share/${PROJECT_NAME}/)
