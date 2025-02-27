/**
 * @file pix_driver.cpp
 * @author Jacob Chisholm (https://jchisholm.github.io)
 * @brief PiCarX Driver
 * @date 2025-02-26
 * @version 0.1
 *
 */

#include "pix_driver.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

#define EXPORT_PATH "/sys/class/gpio/export"
#define UNEXPORT_PATH "/sys/class/gpio/unexport"
#define GPIO_PATH "/sys/class/gpio/gpio%d/"
#define DIRECTION_PATH GPIO_PATH "direction"
#define VALUE_PATH GPIO_PATH "value"

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
    this->turn_offset = 0;
    this->turn_angle = 0;
    this->drive_power = 0;
    this->lift_angle = 0;
    this->camera_tilt = 0;
    this->camera_pan = 0;

    // Setup PWM
    this->setup_pwm(50);
    // Setup Drive Direction Pins
    this->gpio_init(pin_driveDir[0], true);
    this->gpio_init(pin_driveDir[1], true);
}

PiX::~PiX(){
    // Close the I2C Device Connection on Class Deconstruction
    close(this->i2c_fd);
}

void PiX::set_turnAngle(float angle){

}

void PiX::set_turnOffset(float offset_angle){}

float PiX::get_turnAngle(void){
    return this->turn_angle;
}

void PiX::set_drivePower(int power){
    if(power > 0){
        gpio_write(pin_driveDir[0], true);
        gpio_write(pin_driveDir[1], false);
    }
    else{
        gpio_write(pin_driveDir[0], false);
        gpio_write(pin_driveDir[1], true);
    }
    this->i2c_write(PiX::pwm_base + pin_drivePow[0], (int)((power/100.0)*65535));
    this->i2c_write(PiX::pwm_base + pin_drivePow[1], (int)((power/100.0)*65535));
}

int PiX::get_drivePower(void){
    return this->drive_power;
}

void PiX::set_liftAngle(float angle){
}

float PiX::get_liftAngle(void){
    return this->lift_angle;
}

void PiX::set_cameraTilt(float angle){
}

float PiX::get_cameraTilt(void){
    return this->camera_tilt;
}

void PiX::set_cameraPan(float angle){

}

float PiX::get_cameraPan(void){
    return this->camera_pan;
}

float PiX::get_distance(void){
    return 0.0;
}

float PiX::get_lineAverage(void){
    return 0.0;
}

float PiX::get_lineDeviation(void){
    return 0.0;
}

uint32_t PiX::deg_to_pwm(float deg, float max_deg){
    return (uint32_t)((max_deg/2 + deg)*PiX::pwm_max/max_deg);
}

float PiX::adc_to_volt(uint32_t adc_reading){
    return adc_reading*3.3/4095;
}

void PiX::setup_pwm(float freq){
    int psc, arr;
    double best_error = 1e9;
    
    for (int p = 1; p < 256; p++) {
        int a = (int)(PiX::pwm_clk/ (freq * p));
        double error = fabs(freq - (PiX::pwm_clk/ (p * a)));
        if (error < best_error) {
            best_error = error;
            psc = p;
            arr = a;
        }
    }

    i2c_write(PiX::pwm_psc, psc - 1);
    i2c_write(PiX::pwm_arr, arr);
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

int PiX::gpio_init(int pin, bool output){
    char path[64];
    int fd;

    // Export the GPIO pin
    fd = open(EXPORT_PATH, O_WRONLY);
    if (fd < 0) {
        perror("Failed to open GPIO export");
        return -1;
    }
    dprintf(fd, "%d", pin);
    close(fd);

    // Set the direction
    snprintf(path, sizeof(path), DIRECTION_PATH, pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to set GPIO direction");
        return -1;
    }
    if (output)
        write(fd, "out", 3);
    else
        write(fd, "in", 2);
    close(fd);

    return 0;
}

int PiX::gpio_write(int pin, bool value){
    char path[64];
    int fd;

    snprintf(path, sizeof(path), VALUE_PATH, pin);
    fd = open(path, O_WRONLY);
    if (fd < 0) {
        perror("Failed to set GPIO value");
        return -1;
    }
    dprintf(fd, "%d", value);
    close(fd);

    return 0;
}


