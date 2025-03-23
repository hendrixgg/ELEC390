/**
 * @file linetracker.cpp
 * @author Jacob Chisholm
 * @brief PiCarX Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include "roadline_tracker/linetracker.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

LineTracker::LineTracker() : Node("line_tracker"){
    this->declare_parameter("image_topic", "/pix_camera/image_raw");
    this->declare_parameter("output_topic", "line_deviation");
    
    std::string image_topic = this->get_parameter("image_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();

    this->image_sub = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10,
            std::bind(&LineTracker::cv_callback, this, std::placeholders::_1)
            );
    image_topic.append("_processed");
    this->image_pub = this->create_publisher<sensor_msgs::msg::Image>(image_topic, 10);
    this->line_dev_pub = this->create_publisher<std_msgs::msg::Float32>(output_topic, 10);
    this->cam_tilt_pub = this->create_publisher<std_msgs::msg::Float32>("pix_tilt", 10);
    this->cam_pan_pub = this->create_publisher<std_msgs::msg::Float32>("pix_pan", 10);
    RCLCPP_INFO(this->get_logger(), "%s ONLINE", this->get_name());
    // Set the camera angle on repeat
    std_msgs::msg::Float32 tilt_msg;
    tilt_msg.data = 28;
    this->cam_tilt_pub->publish(tilt_msg);
    std_msgs::msg::Float32 pan_msg;
    pan_msg.data = -16;
    this->cam_pan_pub->publish(pan_msg);
}

void LineTracker::cv_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    cv::Mat frame;
    try{
        // Convert ROS2 Image message to OpenCV Mat (YUYV format)
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "yuv422_yuy2"); // Load only Y channel

        // Convert YUYV to BGR using OpenCV
        cv::cvtColor(cv_ptr->image, frame, cv::COLOR_YUV2BGR_YUY2);
    }
    catch (const cv_bridge::Exception &e){
        RCLCPP_ERROR(this->get_logger(), "Failure: %s", e.what());
    }
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    cv::medianBlur(gray, gray, 9);

    // Threshold the image
    cv::Mat thresholded;
    cv::threshold(gray, thresholded, 80, 255, cv::THRESH_BINARY);

    cv::Mat cv_out;
    cv::cvtColor(thresholded, cv_out, cv::COLOR_GRAY2BGR);

    // Compute image moments
    cv::Moments moments = cv::moments(thresholded, true);
    cv::Point center;

    // Check if the moment is valid (avoid division by zero)
    if (moments.m00 > 0) {
        center.x = static_cast<int>(moments.m10 / moments.m00);
        center.y = static_cast<int>(moments.m01 / moments.m00);

        // Draw a dot at the centroid
        cv::circle(cv_out, center, 5, cv::Scalar(0, 255, 255), -1);
    }

    // Find the white line using HoughLinesP
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(thresholded, lines, 1, CV_PI / 180, 50, 50, 10);
    if (!lines.empty()) {
        cv::Vec4i l = lines[0];  // Take the first detected line
        int x_center = (l[0] + l[2]) / 2;
        int y_center = (l[1] + l[3]) / 2;

        // Draw the detected line
        cv::line(cv_out, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0, 255, 0), 2);

        // Blend Hough & Moments
        cv::Point final_center = cv::Point((center.x + x_center) / 2, (center.y + y_center) / 2);

        // Draw a dot at the final center
        cv::circle(cv_out, final_center, 5, cv::Scalar(255, 0, 0), -1);

        // Publish the deviation from the center of the image
        std_msgs::msg::Float32 line_dev_msg;
        line_dev_msg.data = final_center.y - static_cast<float>(msg->height) / 2.0;
        line_dev_msg.data -= final_center.x - static_cast<float>(msg->width) / 2.0;
        this->line_dev_pub->publish(line_dev_msg);
    }


    // Publish the result image for debugging
    cv_bridge::CvImage out_image = cv_bridge::CvImage();
    out_image.header = msg->header;
    out_image.encoding = sensor_msgs::image_encodings::BGR8;
    out_image.image = cv_out;
    this->image_pub->publish(*out_image.toImageMsg());

    // Set the camera angle on repeat
    std_msgs::msg::Float32 tilt_msg;
    tilt_msg.data = -28;
    this->cam_tilt_pub->publish(tilt_msg);
    std_msgs::msg::Float32 pan_msg;
    pan_msg.data = -16;
    this->cam_pan_pub->publish(pan_msg);
}




