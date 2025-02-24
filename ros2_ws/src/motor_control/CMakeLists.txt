cmake_minimum_required(VERSION 3.5)
project(motor_control)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(motor_controller 
    src/main.cpp
)
target_include_directories(motor_controller 
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)

target_link_libraries(motor_controller pigpio)

ament_target_dependencies(motor_controller rclcpp std_msgs)

install(TARGETS
  motor_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

