/**
 * @file driver.hpp
 * @author Jacob Chisholm
 * @brief Driver Node
 * @date 2025-02-27
 * @version 0.1
 *
 */

#ifndef _DRIVER_HPP_
#define _DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"

class Driver : public rclcpp::Node {
    public:
        Driver();
        ~Driver();
        enum eState {
            eState_Driving,
            eState_Blocked,
            eState_Waiting,
            eState_Turn_Right,
            eState_Turn_Left,
            eState_Turn_Straight
        };
        std::unordered_map<enum eState, std::string> stateString = {
            {eState_Driving, "Driving"},
            {eState_Blocked, "Blocked"},
            {eState_Waiting, "Waiting"},
            {eState_Turn_Right, "Turn Right"},
            {eState_Turn_Left, "Turn Left"},
            {eState_Turn_Straight, "Going Straight"},
        };
    private:
        void change_state(enum eState state);
        void line_dev_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void distance_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void line_block_callback(const std_msgs::msg::Float32::SharedPtr msg);
        void rs_obs_callback(const sensor_msgs::msg::PointCloud::SharedPtr msg);
        void state_callback(const std_msgs::msg::String::SharedPtr msg);
        void timer_callback(void);
        void intersection_timer_callback();
        rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr line_dev_sub,
                                                                distance_sub,
                                                                line_sub;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr drive_pow_pub,
                                                             turn_pub;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub;
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_sub;
        rclcpp::Subscription<sensor_msgs::msg::PointCloud>::SharedPtr rs_obs_sub;
        rclcpp::TimerBase::SharedPtr timer, intersection_timer;
        
        float error, error_last, error_sum;
        enum eState state, state_prev_d, state_prev_l;
        float drive_pow;
        float turn_angle;
        int max_area_avg;
        // Parameters
        float param_drive_power;
        float param_left_turn_angle;
        float param_pid_p;
        float param_pid_i;
        float param_pid_d;
        float param_line_trig;
        float param_turn_factor;
        int param_intersection_time;
        rclcpp::Time last_intersection;
};

#endif


