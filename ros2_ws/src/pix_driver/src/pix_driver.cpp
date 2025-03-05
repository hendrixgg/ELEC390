/**
 * @file pix_driver.cpp
 * @author Jacob Chisholm (https://jchisholm.github.io)
 * @brief PiCarX Driver
 * @date 2025-02-26
 * @version 0.1
 *
 */

#include "pix_driver/pix_driver.hpp"
#include <stdio.h>
#include <iostream>
#include <cstdio>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#ifndef TEST
#include <gpiod.h>
#endif
#include <math.h>

PiX::PiX(void){

    // Attempt to open the I2C Device
    this->i2c_fd = open("/dev/i2c-1", O_RDWR);
    if(this->i2c_fd < 0){
        perror("Failed to open I2C Device");
        return;
    }

    // Set I2C Slave address
    if(ioctl(this->i2c_fd, I2C_SLAVE, PiX::i2c_addr) < 0){
        perror("Failed to set I2C slave address");
        return;
    }
    
    // Initialize Default Values
#ifndef TEST
    std::fill_n(this->gpio_lines, 32, nullptr);
#endif
    this->turn_offset = 0;
    this->turn_angle = 0;
    this->drive_power = 0;
    this->lift_angle = 0;
    this->camera_tilt = 0;
    this->camera_pan = 0;

    // Setup PWM
    this->pwm_set_frequency(pin_drivePow[0], 50);
    this->pwm_set_frequency(pin_drivePow[1], 50);
    // Setup Servo PWM
    for(int i = 0; i < 7; i++)
        this->pwm_set_frequency(i, 50);
    // Setup Drive Direction Pins
    this->gpio_lib_init();
    this->gpio_init(pin_driveDir[0], true);
    this->gpio_init(pin_driveDir[1], true);
    this->gpio_init(pin_ultrasonic_trig, true);   // TRIG as OUTPUT
    this->gpio_init(pin_ultrasonic_echo, false);  // ECHO as INPUT

}

PiX::~PiX(){
    this->set_drivePower(0);
    // Close the I2C Device Connection on Class Deconstruction
    close(this->i2c_fd);
#ifndef TEST
    // Close GPIO handles
    if (this->gpio_chip) {
        for (int i = 0; i < 32; i++) {
            if (this->gpio_lines[i]) {
                gpiod_line_release(this->gpio_lines[i]);
            }
        }
        gpiod_chip_close(this->gpio_chip);
    }
#endif
}

void PiX::set_turnAngle(float angle){
    this->turn_angle = angle;
    angle += this->turn_offset;
    uint32_t pwm = turn_min_pwm + ((angle + turn_max_deg) * (turn_max_pwm - turn_min_pwm)) / (2 * turn_max_deg);
    pwm = pwm > turn_max_pwm ? turn_max_pwm : pwm;
    pwm = pwm < turn_min_pwm ? turn_min_pwm : pwm;
    this->pwm_set_pulse_width(pin_turn, pwm);
}

void PiX::set_turnOffset(float offset_angle){
    this->turn_offset = offset_angle;
    // Apply offset
    this->set_turnAngle(this->turn_angle);
}

float PiX::get_turnAngle(void){
    return this->turn_angle;
}

void PiX::set_drivePower(int power){
    if(power > 0){
        gpio_write(pin_driveDir[0], false);
        gpio_write(pin_driveDir[1], true);
    }
    else{
        gpio_write(pin_driveDir[0], true);
        gpio_write(pin_driveDir[1], false);
        power = -power;
    }
    this->i2c_write(PiX::REG_CHN + pin_drivePow[0], (int)((power/100.0)*2000));
    this->i2c_write(PiX::REG_CHN + pin_drivePow[1], (int)((power/100.0)*2000));
}

void PiX::set_drivePower(int pow_left, int pow_right){
    if(pow_left > 0){
        gpio_write(pin_driveDir[0], false);
    }
    else{
        gpio_write(pin_driveDir[0], true);
        pow_left = -pow_left;
    }

    if(pow_right > 0){
        gpio_write(pin_driveDir[1], true);
    }
    else{
        gpio_write(pin_driveDir[1], false);
        pow_right = -pow_right;
    }
    this->i2c_write(PiX::REG_CHN + pin_drivePow[0], (int)((pow_left/100.0)*drive_pwm_max));
    this->i2c_write(PiX::REG_CHN + pin_drivePow[1], (int)((pow_right/100.0)*drive_pwm_max));
}

int PiX::get_drivePower(void){
    return this->drive_power;
}

void PiX::set_liftAngle(float angle){
    this->lift_angle = angle;
    uint32_t pwm = lift_min_pwm + (lift_max_pwm-lift_min_pwm)*angle/lift_max_deg;
    pwm = pwm > lift_max_pwm ? lift_max_pwm : pwm;
    pwm = pwm < lift_min_pwm ? lift_min_pwm : pwm;
    this->pwm_set_pulse_width(pin_lift[0], pwm);
    pwm = lift_max_pwm + lift_min_pwm - pwm;
    this->pwm_set_pulse_width(pin_lift[1], pwm);
}

float PiX::get_liftAngle(void){
    return this->lift_angle;
}

void PiX::set_cameraTilt(float angle){
    angle += this->cam_off_deg;
    this->camera_tilt = angle;
    uint32_t pwm = cam_min_pwm + ((angle + cam_max_deg) * (cam_max_pwm - cam_min_pwm)) / (2 * cam_max_deg);
    pwm = pwm > cam_max_pwm ? cam_max_pwm : pwm;
    pwm = pwm < cam_min_pwm ? cam_min_pwm : pwm;
    this->pwm_set_pulse_width(pin_camera[0], pwm);
}

float PiX::get_cameraTilt(void){
    return this->camera_tilt;
}

void PiX::set_cameraPan(float angle){
    angle += this->cam_off_deg;
    this->camera_pan = angle;
    uint32_t pwm = cam_min_pwm + ((angle + cam_max_deg) * (cam_max_pwm - cam_min_pwm)) / (2 * cam_max_deg);
    pwm = pwm > cam_max_pwm ? cam_max_pwm : pwm;
    pwm = pwm < cam_min_pwm ? cam_min_pwm : pwm;
    this->pwm_set_pulse_width(pin_camera[1], pwm);
}

float PiX::get_cameraPan(void){
    return this->camera_pan;
}

#ifdef TEST
float PiX::get_distance(void) {return 0.0;}
#endif
#ifndef TEST
float PiX::get_distance(void) {
    const float SOUND_SPEED = 343.3;  // Speed of sound in m/s
    const float TIMEOUT = 0.02;       // Timeout in seconds

    // Ensure GPIOs are initialized
    if (!this->gpio_chip) {
        std::cerr << "Error: GPIO chip not initialized. Call gpio_lib_init() first.\n";
        return -1;
    }

    // Send trigger pulse
    gpio_write(pin_ultrasonic_trig, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
    gpio_write(pin_ultrasonic_trig, true);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
    gpio_write(pin_ultrasonic_trig, false);

    // Measure pulse duration
    auto timeout_start = std::chrono::steady_clock::now();
    auto pulse_start = timeout_start;
    auto pulse_end = timeout_start;

    // Wait for echo to go HIGH
    while (gpiod_line_get_value(this->gpio_lines[pin_ultrasonic_echo]) == 0) {
        pulse_start = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(pulse_start - timeout_start).count() > TIMEOUT)
            return -1;  // Timeout error
    }

    // Wait for echo to go LOW
    while (gpiod_line_get_value(this->gpio_lines[pin_ultrasonic_echo]) == 1) {
        pulse_end = std::chrono::steady_clock::now();
        if (std::chrono::duration<float>(pulse_end - timeout_start).count() > TIMEOUT)
            return -1;  // Timeout error
    }

    float duration = std::chrono::duration<float>(pulse_end - pulse_start).count();
    float distance_cm = (duration * SOUND_SPEED / 2) * 100;  // Convert to cm

    return distance_cm;
}
#endif


float PiX::get_lineAverage(void){
    float sum = get_lineSensor(0);
    sum += get_lineSensor(1);
    sum += get_lineSensor(2);
    return sum/3;
}

float PiX::get_lineSensor(int sensor){
    if(sensor < 0 || sensor > 2) return 0.0;
    int reg = 7 - pin_lineFollow[sensor];
    int adc_reading = i2c_read(reg | adc_base);
    return adc_to_volt(adc_reading);
}

float PiX::adc_to_volt(uint32_t adc_reading){
    return adc_reading*3.3/4095;
}

int PiX::i2c_read(int reg){
    unsigned char buffer[2];
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("I2C write (for read) failed");
        return -1;
    }
    if (read(i2c_fd, buffer, 2) != 2) {
        perror("I2C read failed");
        return -2;
    }
    return (buffer[0] << 8) | buffer[1];
}

int PiX::i2c_write(int reg, int value){
    unsigned char buffer[3];
    buffer[0] = reg;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;
    if (write(i2c_fd, buffer, 3) != 3) {
        perror("I2C write failed");
        return -1;
    }
    return 0;
}

void PiX::pwm_set_frequency(int channel, float freq) {
    if (channel < 0 || channel > 19) {
        return;
    }

    int timer_index;
    if (channel < 16)
        timer_index = channel / 4;
    else if (channel == 16 || channel == 17)
        timer_index = 4;
    else if (channel == 18)
        timer_index = 5;
    else
        timer_index = 6;

    int best_psc = 1, best_arr = 1;
    float min_error = CLOCK;
    int start = std::max(1, static_cast<int>(std::sqrt(CLOCK / freq)) - 5);

    for (int psc = start; psc < start + 10; ++psc) {
        int arr = static_cast<int>(CLOCK / (freq * psc));
        float error = std::abs(freq - CLOCK / (psc * arr));
        if (error < min_error) {
            min_error = error;
            best_psc = psc;
            best_arr = arr;
        }
    }

    pwm_set_prescaler(timer_index, best_psc);
    pwm_set_period(timer_index, best_arr);
}

void PiX::pwm_set_prescaler(int timer_index, int prescaler) {
    if (timer_index < 0 || timer_index >= 7) return;

    int reg = (timer_index < 4) ? (REG_PSC + timer_index) : (REG_PSC2 + timer_index - 4);
    i2c_write(reg, prescaler - 1);
}

void PiX::pwm_set_period(int timer_index, int period) {
    if (timer_index < 0 || timer_index >= 7) return;

    timer_arr[timer_index] = period;
    int reg = (timer_index < 4) ? (REG_ARR + timer_index) : (REG_ARR2 + timer_index - 4);
    i2c_write(reg, period);
}

void PiX::pwm_set_pulse_width(int channel, int pulse_width) {
    if (channel < 0 || channel > 19) {
        return;
    }

    int reg = REG_CHN + channel;
    i2c_write(reg, pulse_width);
}

#ifndef TEST
int PiX::gpio_lib_init() {
    this->gpio_chip = gpiod_chip_open_by_name("gpiochip0");
    if (!this->gpio_chip) {
        perror("Failed to open GPIO chip");
        return -1;
    }
    // printf("GPIO library initialized successfully\n");
    return 0;
}

int PiX::gpio_init(int pin, bool output) {
    if (!this->gpio_chip) {
        std::cerr << "Error: GPIO chip not initialized. Call gpio_library_init() first.\n";
        return -1;
    }

    if (pin < 0 || pin >= 32) {
        std::cerr << "Invalid pin number: " << pin << std::endl;
        return -1;
    }

    this->gpio_lines[pin] = gpiod_chip_get_line(this->gpio_chip, pin);
    if (!this->gpio_lines[pin]) {
        perror("Failed to get GPIO line");
        return -1;
    }

    // Release line if it's already requested
    if (gpiod_line_is_requested(this->gpio_lines[pin])) {
        // printf("GPIO %d is already requested, releasing it.\n", pin);
        gpiod_line_release(this->gpio_lines[pin]);
    }

    struct gpiod_line_request_config config;
    config.consumer = "gpio_control";
    config.request_type = output ? GPIOD_LINE_REQUEST_DIRECTION_OUTPUT : GPIOD_LINE_REQUEST_DIRECTION_INPUT;
    config.flags = 0;

    if (gpiod_line_request(this->gpio_lines[pin], &config, output ? 1 : 0) < 0) {
        perror("Failed to request GPIO line");
        return -1;
    }

    // printf("GPIO %d initialized as %s\n", pin, output ? "OUTPUT" : "INPUT");
    return 0;
}

int PiX::gpio_write(int pin, bool value) {
    if (!this->gpio_chip) {
        std::cerr << "Error: GPIO chip not initialized.\n";
        return -1;
    }

    if (!this->gpio_lines[pin]) {
        std::cerr << "Error: GPIO line not initialized for pin " << pin << ".\n";
        return -1;
    }

    if (gpiod_line_set_value(this->gpio_lines[pin], value) < 0) {
        perror("Failed to set GPIO value");
        return -1;
    }

    return 0;
}

#endif

#ifdef TEST
int PiX::gpio_lib_init(void){
    return 0;
}
int PiX::gpio_init(int pin, bool output){
    return 0;
}
int PiX::gpio_write(int pin, bool value){
    return 0;
}

#endif


