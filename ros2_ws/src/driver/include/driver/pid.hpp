/**
 * @file pid.hpp
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
#include "sensor_msgs/msg/image.hpp"

class PID {
    public:
        PID(float kp, float ki, float kd, float min, float max);

};

#endif



