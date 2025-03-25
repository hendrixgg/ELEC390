/**
 * @file obstacle_avoider.hpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */

#ifndef _OBSTACLE_AVOIDER_HPP_
#define _OBSTACLE_AVOIDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/opencv.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class ObstacleAvoider : public rclcpp::Node {
    public:
        ObstacleAvoider();
    private:
        void color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void infra_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void process_images();

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr infra_sub;

        cv::Mat color;
        cv::Mat depth;
        cv::Mat infra;

        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr obstacles_pub;

};

#endif

