/**
 * @file obsticle_avoider.hpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */

#ifndef _OBSTICLE_AVOIDER_HPP_
#define _OBSTICLE_AVOIDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "opencv2/opencv.hpp"

class ObsticleAvoider : public rclcpp::Node {
    public:
        ObsticleAvoider();
    private:
        void color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
        void process_images();

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_sub;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub;

        cv::Mat color;
        cv::Mat depth;

        std::mutex data_mutex_;  // To protect last_color_ and last_depth_
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_pub;

};

#endif

