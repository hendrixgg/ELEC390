/**
 * @file linetracker.hpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */

#ifndef _LINETRACKER_HPP_
#define _LINETRACKER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"

class LineTracker : public rclcpp::Node {
    public:
        LineTracker();
    private:
        void cv_callback(const sensor_msgs::msg::Image::SharedPtr msg);
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_dev_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cam_tilt_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr cam_pan_pub;

};

#endif

