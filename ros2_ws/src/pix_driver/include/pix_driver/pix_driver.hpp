/**
 * @file pix_driver.hpp
 * @author Jacob Chisholm (https://jchisholm.github.io)
 * @brief PiCarX Driver Header File
 * @date 2025-02-26
 * @version 0.1
 *
 */

#ifndef _PIX_DRIVER_H_
#define _PIX_DRIVER_H_

// Uncomment to replace the Raspberry Pi GPIO Library with Null Functions
// #define TEST

#include <stdint.h>
#ifndef TEST
#include <gpiod.h>
#endif
#include <cmath>
#include <iostream>
#include <chrono>
#include <thread>

class PiX {
    public:
        // Class Constructor
        PiX(void);
        // Class Deconstructor
        ~PiX(void);

        /**
         * @brief Set the Drive Turn Angle
         *
         * @param angle Angle to turn to (-30 to +30 deg)
         */
        void set_turnAngle(float angle);

        /**
         * @brief Set the turn offset
         *
         * @param offset_angle applied to each turnAngle command
         */
        void set_turnOffset(float offset_angle);

        /**
         * @brief Get the current turn angle
         *
         * @return Current turn angle in degrees
         */
        float get_turnAngle(void);

        /**
         * @brief Set the Drive Power
         *
         * @param power -100 to 100
         */
        void set_drivePower(int power);

        /**
         * @brief Set the Drive Power
         *
         * @param power -100 to 100
         */
        void set_drivePower(int pow_left, int pow_right);

        /**
         * @brief Get the current Drive Power
         *
         * @returns power -100 to 100
         */
        int get_drivePower(void);

        /**
         * @brief Set the Duck Lift Angle
         *
         * @param angle Angle to set in Degrees
         */
        void set_liftAngle(float angle);

        /**
         * @brief Get the Duck Lift Angle
         *
         * @returns angle Angle to set in Degrees
         */
        float get_liftAngle(void);

        /**
         * @brief Set the Camera Tilt Angle
         *
         * @param angle Angle in degrees to set
         */
        void set_cameraTilt(float angle);

        /**
         * @brief Get the Camera Tilt Angle
         *
         * @returns The Current Angle in Degrees
         */
        float get_cameraTilt(void);

        /**
         * @brief Set the Camera Pan Angle
         *
         * @param angle Angle in degrees to set
         */
        void set_cameraPan(float angle);

        /**
         * @brief Get the Camera Pan Angle
         *
         * @returns The Current Pan angle in degrees
         */
        float get_cameraPan(void);

        /**
         * @brief Get the Ultrasonic Distance in centimeters
         *
         * @return 
         */
        float get_distance(void);

        /**
         * @brief Get the Average Voltage reading from the line sensors
         */
        float get_lineAverage(void);

        float get_lineSensor(int sensor);

    private:
        // Internal References
        float turn_offset;
        float turn_angle;
        int drive_power;
        float lift_angle;
        float camera_tilt;
        float camera_pan;

        // GPIO Handles
#ifndef TEST
        struct gpiod_chip *gpio_chip;
        struct gpiod_line *gpio_lines[32];
#endif

        // I2C Device Conversion Functions
        float    adc_to_volt(uint32_t adc_reading);

        // Device Setup Functions
        void setup_pwm(float freq);

        // I2C Device Interfacing
        int i2c_fd;
        int i2c_read(int reg);
        int i2c_write(int reg, int value);
        void pwm_set_frequency(int channel, float freq);
        void pwm_set_prescaler(int timer_index, int prescaler);
        void pwm_set_period(int timer_index, int period);
        void pwm_set_pulse_width(int channel, int pulse_width);
        int timer_arr[7] = {1, 1, 1, 1, 1, 1, 1};  // Stores period for each timer

        // GPIO Interface Functions
        int gpio_lib_init(void);
        int gpio_init(int pin, bool output);
        int gpio_write(int pin, bool value);

        // I2C Device Information
        constexpr static int i2c_addr = 0x14;
        static constexpr float CLOCK = 72000000.0;  // 72 MHz clock
        static constexpr int REG_CHN = 0x20;
        static constexpr int REG_PSC = 0x40;
        static constexpr int REG_ARR = 0x44;
        static constexpr int REG_PSC2 = 0x50;
        static constexpr int REG_ARR2 = 0x54;
        constexpr static int adc_base = 0x10;

        // Constants used when interfacing with the PiX Hat
        constexpr static int drive_pwm_max = 2000;

        // Device Pin Mappings (Note: Some are digital, some are pwm, some are analog)
        constexpr static int pin_lineFollow[3] = {0, 1, 2};
        // BCM Pins (Raspberry Pi)
        constexpr static int pin_driveDir[2] = {23, 24};
        // Hat Pins
        constexpr static int pin_drivePow[2] = {13, 12};
        constexpr static int pin_turn = 0;
        // Trigger / Echo Pins
        constexpr static int pin_ultrasonic_trig = 27;
        constexpr static int pin_ultrasonic_echo = 22;
        constexpr static int pin_lift[2] = {4, 5};
        constexpr static int pin_camera[2] = {1, 2};

        // Device Constants
        constexpr static float turn_max_deg = 30;
        constexpr static uint32_t turn_min_pwm = 60;
        constexpr static uint32_t turn_max_pwm = 105;

        constexpr static float cam_max_deg = 30;
        constexpr static float cam_off_deg = -11.7;
        constexpr static uint32_t cam_min_pwm = 35;
        constexpr static uint32_t cam_max_pwm = 195;

        constexpr static float lift_max_deg = 180;
        constexpr static uint32_t lift_min_pwm = 26;
        constexpr static uint32_t lift_max_pwm = 135;
};

#endif

