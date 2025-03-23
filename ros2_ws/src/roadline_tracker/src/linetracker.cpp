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
    cv::medianBlur(gray, gray, 7);

    // Threshold the image
    cv::Mat thresholded;
    cv::threshold(gray, thresholded, 80, 255, cv::THRESH_BINARY);

    cv::Mat cv_out;
    cv::cvtColor(thresholded, cv_out, cv::COLOR_GRAY2BGR);
    
    // Find the white line
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(thresholded, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if(!contours.empty()){
        cv::Rect bounding_box = cv::boundingRect(contours[0]);
        int center_x = bounding_box.x + bounding_box.width/2;
        int center_y = bounding_box.y + bounding_box.height/2;
        std_msgs::msg::Float32 line_dev_msg;
        line_dev_msg.data = center_y-(float)((float)msg->height/2.0);
        this->line_dev_pub->publish(line_dev_msg);

        // Draw on the image
        cv::rectangle(cv_out, bounding_box, cv::Scalar(0, 255, 0), 2);
        // Draw center point in red
        cv::circle(cv_out, cv::Point(center_x, center_y), 5, cv::Scalar(0, 0, 255), -1);
    }

    // Publish the result image for debugging
    cv_bridge::CvImage out_image = cv_bridge::CvImage();
    out_image.header = msg->header;
    out_image.encoding = sensor_msgs::image_encodings::BGR8;
    out_image.image = cv_out;
    this->image_pub->publish(*out_image.toImageMsg());
}




