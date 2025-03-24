# CMake File for Roadline Tracker
# Jacob Chisholm (https://Jchisholm204.github.io)
# version 0.1
# 2025-03-23
set(NODE roadline_tracker)

# Ensure the required packages are avaliable
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)

# Gather all sources for this project
file(GLOB_RECURSE PROJECT_SOURCES FOLLOW_SYMLINKS
    ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/src/*.cpp
)

# Setup Library Dependencies
set(${NODE}_DEPS rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

# Add to include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include)

# Add this node as an executable 
add_executable(${NODE}
    ${PROJECT_SOURCES}
)

target_include_directories(${NODE} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include>
    $<INSTALL_INTERFACE:include>
)

# Library dependencies
ament_target_dependencies(${NODE} ${${NODE}_DEPS})

# Install header files
install(
    DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include/
    DESTINATION include/${PROJECT_NAME}
)

# Install library files
install(
    TARGETS ${NODE}
    EXPORT export_${NODE}
    DESTINATION lib/${PROJECT_NAME}
)

# Exports
ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${NODE}/include)
ament_export_targets(export_${NODE} HAS_LIBRARY_TARGET)
ament_export_dependencies(${${NODE}_DEPS})


