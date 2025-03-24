/**
 * @file linetracker.cpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include "duck_avoidance/obsticle_avoider.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ObsticleAvoider::ObsticleAvoider() : Node("line_tracker"){
    this->declare_parameter("color_topic", "/camera/realsense/color/image_raw");
    this->declare_parameter("depth_topic", "/camera/realsense/infra2/image_rect_raw");
    this->declare_parameter("image_output", "rs_processed");
    this->declare_parameter("output_topic", "rs_min_dist");
    
    std::string color_topic = this->get_parameter("color_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string output_image = this->get_parameter("image_output").as_string();

    // Subscribe to topics (message_filters requires explicit initialization)
    color_sub.subscribe(this, color_topic);
    depth_sub.subscribe(this, depth_topic);

    // Create TimeSynchronizer
    sync_ = std::make_shared<message_filters::Synchronizer<sync_policy>>(
            sync_policy(1), color_sub, depth_sub);

    // Register callback
    sync_->registerCallback(std::bind(&ObsticleAvoider::cv_callback, this,
                std::placeholders::_1, std::placeholders::_2));

    // Publishers
    image_pub = this->create_publisher<sensor_msgs::msg::Image>(output_image, 10);
    min_distance_pub = this->create_publisher<std_msgs::msg::Float32>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "%s ONLINE", this->get_name());

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = -16;
    this->min_distance_pub->publish(dist_msg);
}

void ObsticleAvoider::cv_callback(const sensor_msgs::msg::Image::ConstSharedPtr& color,
        const sensor_msgs::msg::Image::ConstSharedPtr& depth){
    // cv::Mat frame;
    // try{
    //     // Convert ROS2 Image message to OpenCV Mat (YUYV format)
    //     cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(depth, "mono8"); // Load only Y channel
    //
    //     // Convert YUYV to BGR using OpenCV
    //     cv::cvtColor(cv_ptr->image, frame, cv::COLOR_GRAY2BGR);
    // }
    // catch (const cv_bridge::Exception &e){
    //     RCLCPP_ERROR(this->get_logger(), "Failure: %s", e.what());
    // }
    RCLCPP_INFO(this->get_logger(), "Hello");
    // cv::Mat gray;
    // cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    // cv::medianBlur(gray, gray, 9);
    //
    // // Publish the result image for debugging
    // cv_bridge::CvImage out_image = cv_bridge::CvImage();
    // out_image.header = color->header;
    // out_image.encoding = sensor_msgs::image_encodings::BGR8;
    // out_image.image = frame;
    // this->image_pub->publish(*out_image.toImageMsg());
    this->image_pub->publish(*color);
}




