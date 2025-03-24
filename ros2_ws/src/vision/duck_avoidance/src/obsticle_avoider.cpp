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

     // Subscribe to color and depth images
    this->color_sub = this->create_subscription<sensor_msgs::msg::Image>(
        color_topic, 10,
        std::bind(&ObsticleAvoider::color_callback, this, std::placeholders::_1));

    this->depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
        depth_topic, 10,
        std::bind(&ObsticleAvoider::depth_callback, this, std::placeholders::_1));

    // Publishers
    image_pub = this->create_publisher<sensor_msgs::msg::Image>(output_image, 10);
    min_distance_pub = this->create_publisher<std_msgs::msg::Float32>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "%s ONLINE", this->get_name());

    std_msgs::msg::Float32 dist_msg;
    dist_msg.data = -16;
    this->min_distance_pub->publish(dist_msg);
}

void ObsticleAvoider::color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
    try {
        // Check if image encoding is correct
        // RCLCPP_INFO(this->get_logger(), "Received image with encoding: %s", msg->encoding.c_str());

        // Convert ROS2 Image message to OpenCV Mat
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "rgb8"); 

        // Ensure the image is valid
        if (cv_ptr->image.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Received empty image!");
            return;
        }

        // Store the image
        this->color = cv_ptr->image;
    } 
    catch (const cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Standard exception: %s", e.what());
        return;
    }

    process_images();
}

void ObsticleAvoider::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    try{
        // Convert ROS2 Image message to OpenCV Mat (YUYV format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8"); // Load only Y channel

        // Convert YUYV to BGR using OpenCV
        cv::cvtColor(cv_ptr->image, this->depth, cv::COLOR_GRAY2BGR);
    }
    catch (const cv_bridge::Exception &e){
        RCLCPP_ERROR(this->get_logger(), "Failure: %s", e.what());
    }
    process_images();
}

void ObsticleAvoider::process_images(){
    if(color.empty() || depth.empty()) return;


    // Publish the result image for debugging
    cv_bridge::CvImage out_image = cv_bridge::CvImage();
    std_msgs::msg::Header header;
    header.stamp = this->now();
    out_image.header = header;
    out_image.encoding = sensor_msgs::image_encodings::RGB8;
    out_image.image = color;
    this->image_pub->publish(*out_image.toImageMsg());
    out_image.image = depth;
    this->image_pub->publish(*out_image.toImageMsg());
}



