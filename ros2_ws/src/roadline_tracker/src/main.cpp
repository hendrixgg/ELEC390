#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float32.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "roadline_tracker/linetracker.hpp"

class LineTrackerNode : public rclcpp::Node {
public:
    LineTrackerNode() : Node("line_tracker") {
        this->declare_parameter("image_topic", "/camera/image_raw");
        this->declare_parameter("output_topic", "line_position");

        std::string image_topic = this->get_parameter("image_topic").as_string();
        std::string output_topic = this->get_parameter("output_topic").as_string();

        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            image_topic, 10, 
            std::bind(&LineTrackerNode::image_callback, this, std::placeholders::_1)
        );

        line_pub_ = this->create_publisher<std_msgs::msg::Float32>(output_topic, 10);
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
            
            // Convert to grayscale
            cv::Mat gray;
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

            // Threshold to detect white areas (adjust threshold as needed)
            cv::Mat binary;
            cv::threshold(gray, binary, 200, 255, cv::THRESH_BINARY);

            // Find contours
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(binary, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            if (!contours.empty()) {
                // Find largest contour (assumed to be the line)
                std::vector<cv::Point> largest_contour = *std::max_element(
                    contours.begin(), contours.end(),
                    [](const std::vector<cv::Point>& a, const std::vector<cv::Point>& b) {
                        return cv::contourArea(a) < cv::contourArea(b);
                    }
                );
                
                // Compute centroid of the largest contour
                cv::Moments m = cv::moments(largest_contour);
                if (m.m00 != 0) {
                    float cx = static_cast<float>(m.m10 / m.m00);
                    
                    // Publish x-coordinate of the centroid
                    auto msg = std_msgs::msg::Float32();
                    msg.data = cx;
                    line_pub_->publish(msg);
                }
            }
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge Error: %s", e.what());
        }
    }
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr line_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LineTracker>());
    rclcpp::shutdown();
    return 0;
}

