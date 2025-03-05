cmake_minimum_required(VERSION 3.8)
project(roadline_tracker)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(roadline_tracker
    src/main.cpp
)

include_directories(
    include
    /usr/include
)
# target_link_libraries(pix_node PRIVATE gpiod)

# target_include_directories(pix_node include)
target_link_libraries(roadline_tracker
    ${OpenCV_libs}
    # opencv_core
)

ament_target_dependencies(roadline_tracker rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(TARGETS roadline_tracker
  DESTINATION lib/${PROJECT_NAME})
ament_package()
