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

class ObsticleAvoider : public rclcpp::Node {
    public:
        ObsticleAvoider();
    private:
        void cv_callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
                     const sensor_msgs::msg::Image::ConstSharedPtr& depth);
        message_filters::Subscriber<sensor_msgs::msg::Image> color_sub;
        message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub;
        using sync_policy = message_filters::sync_policies::ApproximateTime<
            sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
        std::shared_ptr<message_filters::Synchronizer<sync_policy>> sync_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr min_distance_pub;

};

#endif

