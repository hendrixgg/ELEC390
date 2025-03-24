/**
 * @file driver.cpp
 * @author Jacob Chisholm
 * @brief Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */
#include "driver/driver.hpp"
#include "rclcpp/time.hpp"

Driver::Driver() : Node("driver_node") {
    // Get Parameters
    this->declare_parameter("drive_power", 40.0);
    this->declare_parameter("left_turn_angle", -20.0);
    this->declare_parameter("pid_p", 0.1);
    this->declare_parameter("pid_i", 0.2);
    this->declare_parameter("pid_d", 0.05);
    this->declare_parameter("intersection_time_ms", 300);
    
    this->param_drive_power = this->get_parameter("drive_power").as_double();
    this->param_left_turn_angle = this->get_parameter("left_turn_angle").as_double();
    this->param_pid_p = this->get_parameter("pid_p").as_double();
    this->param_pid_i = this->get_parameter("pid_i").as_double();
    this->param_pid_d = this->get_parameter("pid_d").as_double();
    this->param_intersection_time = this->get_parameter("intersection_time_ms").as_int();

    // Setup Publishers/Subscribers
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

    // System Runtime Timer
    this->timer = this->create_wall_timer(std::chrono::milliseconds(100),
            std::bind(&Driver::timer_callback, this));
    // Intersection Timer (One Shot)
    this->intersection_timer = this->create_wall_timer(
            std::chrono::milliseconds(this->param_intersection_time),
            std::bind(&Driver::intersection_timer_callback, this)
            );

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
    float p = this->param_pid_p*error;
    float i = this->param_pid_i*error_sum;
    float d = this->param_pid_d*derr;
    float pow = p + i + d;
    this->turn_angle = pow;
    this->drive_pow = this->param_drive_power;
}

void Driver::change_state(enum eState state){
    RCLCPP_INFO(this->get_logger(), 
            "CHANGE STATE %s -> %s\n",
            stateString[this->state].c_str(), stateString[state].c_str());
    this->state = state;
}

void Driver::distance_callback(const std_msgs::msg::Float32::SharedPtr msg){
    float cm = msg->data;
    if(cm != -1 && cm < 20){
        if(state != eState_Blocked)
            state_prev_d = state;
        this->change_state(eState_Blocked);
    }
    else if (state == eState_Blocked){
        this->change_state(state_prev_d);
    }

}
void Driver::line_block_callback(const std_msgs::msg::Float32::SharedPtr msg){
    if(msg->data > 1){
        if(state != eState_Waiting)
            state_prev_l = state;
        this->change_state(eState_Waiting);
    }
    else{
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

void Driver::timer_callback(void){
    std_msgs::msg::String state_msg;
    state_msg.data = stateString[state];
    this->state_pub->publish(state_msg);
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
            break;
        case eState_Turn_Left:
            // Fixed turn radius for left turns
            drive_msg.data = drive_pow;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = this->param_left_turn_angle;
            this->turn_pub->publish(turn_msg);
            this->intersection_timer->reset();
            // Start reset timer
            this->intersection_timer->cancel();
            this->intersection_timer->reset();
            break;
        case eState_Turn_Right:
            // Follow the line for right turns
            drive_msg.data = drive_pow;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = turn_angle;
            this->turn_pub->publish(turn_msg);
            break;
        case eState_Turn_Straight:
            // Follow the line for right turns
            drive_msg.data = this->param_drive_power;
            this->drive_pow_pub->publish(drive_msg);
            turn_msg.data = 0;
            this->turn_pub->publish(turn_msg);
            // Start reset timer
            this->intersection_timer->cancel();
            this->intersection_timer->reset();
            break;

    }
}

void Driver::intersection_timer_callback(void){
    // Switch state back to driving after timer elapsed
    switch(this->state){
        case eState_Driving:
        case eState_Blocked:
        case eState_Waiting:
            break;
        case eState_Turn_Left:
        case eState_Turn_Right:
        case eState_Turn_Straight:
            this->state = eState_Driving;
    }
    // Cancel the timer (make it one shot)
    this->intersection_timer->cancel();
}
