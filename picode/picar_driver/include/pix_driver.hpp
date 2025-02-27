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
#define TEST

#include <stdint.h>
#ifndef TEST
#include <gpiod.h>
#endif

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

        /**
         * @brief Get the line sensors deviation from the center
         *
         * @return negative value for left deviation, positive for right
         */
        float get_lineDeviation(void);

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
        uint32_t deg_to_pwm(float deg, float max_deg);
        float    adc_to_volt(uint32_t adc_reading);

        // Device Setup Functions
        void setup_pwm(float freq);

        // I2C Device Interfacing
        int i2c_fd;
        int i2c_read(int reg);
        int i2c_write(int reg, int value);
        int set_pwm(int channel, uint16_t val);

        // GPIO Interface Functions
        int gpio_lib_init(void);
        int gpio_init(int pin, bool output);
        int gpio_write(int pin, bool value);

        // I2C Device Information
        constexpr static int i2c_addr = 0x14;
        constexpr static int pwm_base = 0x20;
        constexpr static int pwm_psc  = 0x40;
        constexpr static int pwm_arr  = 0x44;
        constexpr static int pwm_psc2 = 0x50;
        constexpr static int pwm_arr2 = 0x54;
        constexpr static float pwm_clk  = 72000000.0;
        constexpr static int adc_base = 0x10;

        // Constants used when interfacing with the PiX Hat
        constexpr static int pwm_max = 4095;

        // Device Pin Mappings (Note: Some are digital, some are pwm, some are analog)
        constexpr static int pin_lineFollow[3] = {0, 1, 2};
        // BCM Pins (Raspberry Pi)
        constexpr static int pin_driveDir[2] = {23, 24};
        constexpr static int pin_drivePow[2] = {13, 12};
        constexpr static int pin_turn = 0;
        constexpr static int pin_ultrasonic[2] = {2, 3};
        constexpr static int pin_lift[2] = {3, 4};
        constexpr static int pin_camera[2] = {1, 2};
};

#endif
