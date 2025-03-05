/**
 * @file node.hpp
 * @author Jacob Chisholm (https://jchisholm.github.io)
 * @brief PiCarX Driver
 * @date 2025-02-27
 * @version 0.1
 *
 */
#ifndef _NODE_HPP_
#define _NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "pix_driver/pix_driver.hpp"

class PixNode : public rclcpp::Node {
    public:
        PixNode();
    private:
        void turn_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void drive_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void tilt_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void pan_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void lift_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void timer_callback(void);
        PiX px;
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sub_turn,
                                                                sub_drive,
                                                                sub_tilt,
                                                                sub_pan,
                                                                sub_lift;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_distance;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_line;
        rclcpp::TimerBase::SharedPtr timer;
        float diff_ratio;
};

#endif
