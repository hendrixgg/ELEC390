/**
 * @file main.cpp
 * @author Jacob Chisholm
 * @brief Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include <rclcpp/rclcpp.hpp>
#include "driver/driver.hpp"

// Main function to run the ROS 2 node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Driver>());
    rclcpp::shutdown();
    return 0;
}

