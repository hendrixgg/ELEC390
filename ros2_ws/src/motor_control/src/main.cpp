#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <pigpio.h>

class MotorController : public rclcpp::Node {
public:
    MotorController() : Node("motor_controller") {
        subscription_ = this->create_subscription<std_msgs::msg::Int32>(
            "motor_speed", 10,
            std::bind(&MotorController::motor_callback, this, std::placeholders::_1));
        
        // if (gpioInitialise() < 0) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to initialize pigpio!");
        //     rclcpp::shutdown();
        // }
        gpioInitialise();

        gpioSetPWMfrequency(PWM_PIN, 1000); // Set PWM frequency to 1kHz
        RCLCPP_INFO(this->get_logger(), "Motor controller node started.");
    }

    ~MotorController() {
        gpioPWM(PWM_PIN, 0); // Stop motor
        gpioTerminate();
    }

private:
    void motor_callback(const std_msgs::msg::Int32::SharedPtr msg) {
        int speed = std::clamp(msg->data, 0, 255); // Ensure valid range
        gpioPWM(PWM_PIN, speed);
        RCLCPP_INFO(this->get_logger(), "Set motor speed to: %d", speed);
    }

    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    const int PWM_PIN = 12; // Change this based on your wiring
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorController>());
    rclcpp::shutdown();
    return 0;
}

