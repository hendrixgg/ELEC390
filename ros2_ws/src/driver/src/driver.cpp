/**
 * @file driver.cpp
 * @author Jacob Chisholm
 * @brief Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include "driver/driver.hpp"

Driver::Driver() : Node("driver_node") {
    this->line_dev_sub = this->create_subscription<std_msgs::msg::Float32>("line_deviation", 10,
            std::bind(&Driver::line_dev_callback, this, std::placeholders::_1));
    this->distance_sub = this->create_subscription<std_msgs::msg::Float32>("pix_distance", 10,
            std::bind(&Driver::distance_callback, this, std::placeholders::_1));
    this->line_sub = this->create_subscription<std_msgs::msg::Float32>("pix_line", 10,
            std::bind(&Driver::line_block_callback, this, std::placeholders::_1));
    this->drive_pow_pub = this->create_publisher<std_msgs::msg::Float32>("pix_drive", 10);
    this->turn_pub = this->create_publisher<std_msgs::msg::Float32>("pix_turn", 10);
    this->state_pub = this->create_publisher<std_msgs::msg::String>("state/current", 10);
    this->state_sub = this->create_subscription<std_msgs::msg::String>("state/next", 10,
            std::bind(&Driver::state_callback, this, std::placeholders::_1));
    error = 0;
    error_last = 0;
    error_sum = 0;
    this->state = eState_Driving;
}

Driver::~Driver(){
    std_msgs::msg::Float32 turn_msg;
    turn_msg.data = 0;
    this->turn_pub->publish(turn_msg);

    std_msgs::msg::Float32 drive_msg;
    drive_msg.data = 0;
    this->drive_pow_pub->publish(drive_msg);
}

void Driver::line_dev_callback(const std_msgs::msg::Float32::SharedPtr msg){
    error = -(msg->data-20);
    float derr = (error - error_last);
    error_sum = (error + error_sum)/2;
    error_last = error;
    float p = 0.1*error;
    float i = 0.2*error_sum;
    float d = 0.05*derr;
    float pow = p + i + d;
    this->turn_angle = pow;
    this->drive_pow = 40;

    if(state == eState_Driving)
        this->change_state(eState_Driving);
}

void Driver::change_state(enum eState state){
    RCLCPP_INFO(this->get_logger(), 
            "CHANGE STATE %s -> %s\n",
            stateString[this->state].c_str(), stateString[state].c_str());
    this->state = state;
    // Send out messages to drivetrain
    std_msgs::msg::Float32 drive_msg;
    std_msgs::msg::Float32 turn_msg;
    switch(this->state){
        case eState_Driving:
            drive_msg.data = drive_pow;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = turn_angle;
            this->turn_pub->publish(turn_msg);
            break;
        case eState_Blocked:
        case eState_Waiting:
            drive_msg.data = 0;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = 0;
            this->turn_pub->publish(turn_msg);
        case eState_Turn_Left:
            // Fixed turn radius for left turns
            drive_msg.data = drive_pow;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = -20;
            this->turn_pub->publish(turn_msg);
        case eState_Turn_Right:
            // Follow the line for right turns
            drive_msg.data = drive_pow;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = turn_angle;
            this->turn_pub->publish(turn_msg);
        case eState_Turn_Straight:
            // Follow the line for right turns
            drive_msg.data = 40;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = 0;
            this->turn_pub->publish(turn_msg);

    }
}

void Driver::distance_callback(const std_msgs::msg::Float32::SharedPtr msg){
    float cm = msg->data;
    if(cm != -1 && cm < 10){
        if(state != eState_Blocked)
            state_prev_d = state;
        change_state(eState_Blocked);
    }
    else{
        state = state_prev_d;
        state_prev_d = state;
    }

}
void Driver::line_block_callback(const std_msgs::msg::Float32::SharedPtr msg){
    if(msg->data > 1.2){
        state_prev_l = state;
        state = eState_Waiting;
    }
    else{
        state = state_prev_d;
        state_prev_l = state;
    }
}

void Driver::state_callback(const std_msgs::msg::String::SharedPtr msg){
    enum eState new_state;
    if(stateString[eState_Driving] == msg->data){
        new_state = eState_Driving;
    }
    if(stateString[eState_Blocked] == msg->data){
        new_state = eState_Blocked;
    }
    if(stateString[eState_Waiting] == msg->data){
        new_state = eState_Waiting;
    }
    if(stateString[eState_Turn_Right] == msg->data){
        new_state = eState_Turn_Right;
    }
    if(stateString[eState_Turn_Left] == msg->data){
        new_state = eState_Turn_Left;
    }
    if(stateString[eState_Turn_Straight] == msg->data){
        new_state = eState_Turn_Straight;
    }
    this->change_state(new_state);
}
