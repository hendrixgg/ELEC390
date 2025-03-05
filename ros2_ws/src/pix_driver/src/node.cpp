/**
 * @file node.cpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "pix_driver/pix_driver.hpp"
#include "pix_driver/node.hpp"

PixNode::PixNode() : Node("pix_node"), px() {
    RCLCPP_INFO(this->get_logger(), "PiX Node Initialized");
    // Setup ROS Launch parameters
    this->declare_parameter("turn_offset", 0.0);
    this->declare_parameter("diff_ratio", 0.30);

    // Load ROS parameters
    diff_ratio = this->get_parameter("diff_ratio").as_double();
    if(diff_ratio > 1) diff_ratio = 1;
    if(diff_ratio < 0) diff_ratio = 0;

    float turn_offset = this->get_parameter("turn_offset").as_double();
    px.set_turnOffset(turn_offset);

    // Timer to periodically publish sensor data
    timer = this->create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(&PixNode::timer_callback, this)
            );

    RCLCPP_INFO(this->get_logger(), "PiX Turn Offset = %0.2f deg", turn_offset);
    RCLCPP_INFO(this->get_logger(), "PiX Differential Ratio = %0.2f %%", diff_ratio);

    // Subscribers for various commands
    sub_turn = this->create_subscription<std_msgs::msg::Float32>(
                "pix_turn", 10,
                std::bind(&PixNode::turn_callback, this, std::placeholders::_1)
            );

    sub_drive = this->create_subscription<std_msgs::msg::Float32>(
                "pix_drive", 10,
                std::bind(&PixNode::drive_callback, this, std::placeholders::_1)
            );

    sub_tilt = this->create_subscription<std_msgs::msg::Float32>(
                "pix_tilt", 10,
                std::bind(&PixNode::tilt_callback, this, std::placeholders::_1)
            );

    sub_pan = this->create_subscription<std_msgs::msg::Float32>(
                "pix_pan", 10,
                std::bind(&PixNode::pan_callback, this, std::placeholders::_1)
            );

    sub_lift = this->create_subscription<std_msgs::msg::Float32>(
                "pix_lift", 10,
                std::bind(&PixNode::lift_callback, this, std::placeholders::_1)
            );

    // Publisher for ultrasonic distance
    pub_distance = this->create_publisher<std_msgs::msg::Float32>(
                "pix_distance", 10);
    // Publisher for line sensor
    pub_line = this->create_publisher<std_msgs::msg::Float32>(
                "pix_line", 10);
}

// Callback functions for movement commands
void PixNode::turn_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Setting turn angle to: %.2f", msg->data);
    this->px.set_turnAngle(msg->data);
}

void PixNode::drive_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Setting drive speed to: %.2f", msg->data);
    float left_power = msg->data, right_power = msg->data;
    float turn_angle = this->px.get_turnAngle();
    // Slow right wheel on right turn
    if(turn_angle > 0){
        right_power = diff_ratio*right_power*(30-turn_angle)/30 + (1-diff_ratio)*right_power;
    }
    if(turn_angle < 0){
        turn_angle = -turn_angle;
        left_power = diff_ratio*left_power*(30-turn_angle)/30 + (1-diff_ratio)*left_power;
    }
    this->px.set_drivePower(left_power, right_power);
}

void PixNode::tilt_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Setting tilt angle to: %.2f", msg->data);
    this->px.set_cameraTilt(msg->data);
}

void PixNode::pan_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Setting pan angle to: %.2f", msg->data);
    this->px.set_cameraPan(msg->data);
}

void PixNode::lift_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    // RCLCPP_INFO(this->get_logger(), "Setting lift height to: %.2f", msg->data);
    this->px.set_liftAngle(msg->data);
}

// Timer function to publish sensor data (e.g., ultrasonic distance)
void PixNode::timer_callback() {
    float distance = this->px.get_distance();
    auto msg = std_msgs::msg::Float32();
    msg.data = distance;
    pub_distance->publish(msg);
    msg.data = this->px.get_lineAverage();
    pub_line->publish(msg);
}

// Main function to run the ROS 2 node
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PixNode>());
    rclcpp::shutdown();
    return 0;
}

