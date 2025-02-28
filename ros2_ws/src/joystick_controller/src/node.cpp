#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"

class XboxControllerNode : public rclcpp::Node
{
public:
    XboxControllerNode() : Node("xbox_controller_node")
    {
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&XboxControllerNode::joy_callback, this, std::placeholders::_1));

        pub_turn_ = this->create_publisher<std_msgs::msg::Float32>("pix_turn", 10);
        pub_drive_ = this->create_publisher<std_msgs::msg::Float32>("pix_drive", 10);
        pub_tilt_ = this->create_publisher<std_msgs::msg::Float32>("pix_tilt", 10);
        pub_pan_ = this->create_publisher<std_msgs::msg::Float32>("pix_pan", 10);
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // Steering: Left joystick X-axis (-30 to 30)
        publish_value(pub_turn_, msg->axes[0] * -30.0f);

        // Drive: Right trigger moves forward (0 to 100), left trigger moves backward (-100 to 0)
        float throttle = abs(msg->axes[5] - 1.0f) / 2.0f * 100.0f; // Convert from [-1,1] to [0,100]
        float brake = abs(msg->axes[2] - 1.0f) / 2.0f * 100.0f;    // Convert from [-1,1] to [0,100]
        publish_value(pub_drive_, throttle - brake);

        // Tilt: Right joystick Y-axis (-30 to 30)
        publish_value(pub_tilt_, msg->axes[3] * 30.0f);

        // Pan: Right joystick X-axis (-30 to 30)
        publish_value(pub_pan_, msg->axes[4] * -30.0f);
    }

    void publish_value(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub, float value)
    {
        auto msg = std_msgs::msg::Float32();
        msg.data = value;
        pub->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_turn_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_drive_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_tilt_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_pan_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<XboxControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

