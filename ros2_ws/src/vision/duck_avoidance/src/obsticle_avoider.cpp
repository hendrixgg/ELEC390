/**
 * @file linetracker.cpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include "duck_avoidance/obstacle_avoider.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

ObstacleAvoider::ObstacleAvoider() : Node("obstacle_avoider"){
    this->declare_parameter("color_topic", "/camera/realsense/color/image_raw");
    this->declare_parameter("depth_topic", "/camera/realsense/depth/image_rect_raw");
    this->declare_parameter("infra_topic", "/camera/realsense/infra1/image_rect_raw");
    this->declare_parameter("image_output", "rs_processed");
    this->declare_parameter("output_topic", "rs_obstacles");
    
    std::string color_topic = this->get_parameter("color_topic").as_string();
    std::string depth_topic = this->get_parameter("depth_topic").as_string();
    std::string infra_topic = this->get_parameter("infra_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    std::string output_image = this->get_parameter("image_output").as_string();

     // Subscribe to color and depth images
    // this->color_sub = this->create_subscription<sensor_msgs::msg::Image>(
    //     color_topic, 10,
    //     std::bind(&ObstacleAvoider::color_callback, this, std::placeholders::_1));

    // this->depth_sub = this->create_subscription<sensor_msgs::msg::Image>(
    //     depth_topic, 10,
    //     std::bind(&ObstacleAvoider::depth_callback, this, std::placeholders::_1));

    this->infra_sub = this->create_subscription<sensor_msgs::msg::Image>(
        infra_topic, 10,
        std::bind(&ObstacleAvoider::infra_callback, this, std::placeholders::_1));

    // Publishers
    image_pub = this->create_publisher<sensor_msgs::msg::Image>(output_image, 10);
    obstacles_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "%s ONLINE", this->get_name());

}

void ObstacleAvoider::color_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
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

void ObstacleAvoider::depth_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    try{
        // Convert ROS2 Image message to OpenCV Mat (YUYV format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1"); // Load only Y channel

        // Convert the image to a BGR
        cv_ptr->image.convertTo(this->depth, CV_8UC1);
    }
    catch (const cv_bridge::Exception &e){
        RCLCPP_ERROR(this->get_logger(), "Failure: %s", e.what());
    }
    process_images();
}

void ObstacleAvoider::infra_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg){
    try{
        // Convert ROS2 Image message to OpenCV Mat (YUYV format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "mono8"); // Load only Y channel

        // Convert YUYV to BGR using OpenCV
        this->infra = cv_ptr->image;
        // cv::cvtColor(cv_ptr->image, this->infra, cv::COLOR_GRAY2BGR);
    }
    catch (const cv_bridge::Exception &e){
        RCLCPP_ERROR(this->get_logger(), "Failure: %s", e.what());
    }
    process_images();
}

void ObstacleAvoider::process_images(){
    // if(color.empty() || infra.empty()) return;
    if(infra.empty()) return;
    // RCLCPP_INFO(this->get_logger(), "Publishing Frame");
    // Need to write opencv code that finds the duck
    cv::Mat thresholded_infra;
    cv::medianBlur(infra, thresholded_infra, 17);

    // Create a gradient mask that darkens towards the bottom (same size as input, single channel)
    // This helps remove the floor
    cv::Mat gradient_mask(thresholded_infra.size(), CV_32FC1);
    for (int y = 0; y < gradient_mask.rows; y++) {
        float alpha = 1.0f - (0.70f * y / gradient_mask.rows); // 0.8 means we keep 20% brightness at bottom
        gradient_mask.row(y).setTo(cv::Scalar(alpha));
    }

    // Convert binary image to float (single channel)
    cv::Mat float_infra;
    thresholded_infra.convertTo(float_infra, CV_32FC1, 1.0/255.0);

    // Apply gradient
    cv::Mat result;
    cv::multiply(float_infra, gradient_mask, result);

    // Convert back to 8-bit
    result.convertTo(thresholded_infra, CV_8UC1, 255.0);

    // threshold image
    cv::threshold(thresholded_infra, thresholded_infra, 110, 255, cv::THRESH_BINARY);

    // Find Contours in the Image (Obstacles)
    // Find contours
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresholded_infra, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Draw bounding boxes around each contour
    cv::Mat result_image;
    cv::cvtColor(thresholded_infra, result_image, cv::COLOR_GRAY2BGR);

    // Obstacles Vector
    std::vector<geometry_msgs::msg::Point32> obstacles;

    for (size_t i = 0; i < contours.size(); i++) {
        // Get the bounding rectangle for each contour
        cv::Rect bounding_box = cv::boundingRect(contours[i]);

        // Draw a rectangle around the obstacle
        cv::rectangle(result_image, bounding_box, cv::Scalar(0, 0, 255), 2);
        cv::Moments moments = cv::moments(contours[i]);
        int center_x = static_cast<int>(moments.m10 / moments.m00);
        int center_y = static_cast<int>(moments.m01 / moments.m00);

        // Draw a dot at the center of the obstacle
        cv::circle(result_image, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);

        // Push back to obstacle array
        geometry_msgs::msg::Point32 point;
        point.x = center_x;
        point.y = center_y;
        // Use Z to store area of obstacle
        point.z = bounding_box.x*bounding_box.y;
        obstacles.push_back(point);
    }

    // Create the PointCloud message
    sensor_msgs::msg::PointCloud point_cloud_msg;
    point_cloud_msg.header.stamp = this->now();
    point_cloud_msg.header.frame_id = "none"; 
    point_cloud_msg.points = obstacles;

    // Publish the PointCloud message
    this->obstacles_pub->publish(point_cloud_msg);

    // Publish the result image for debugging
    cv_bridge::CvImage out_image = cv_bridge::CvImage();
    std_msgs::msg::Header header;
    header.stamp = this->now();
    out_image.header = header;
    out_image.encoding = sensor_msgs::image_encodings::BGR8;
    out_image.image = result_image;
    this->image_pub->publish(*out_image.toImageMsg());
    // out_image.image = depth;
    // this->image_pub->publish(*out_image.toImageMsg());
}



