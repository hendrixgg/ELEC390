#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <math.h>

#define I2C_DEV "/dev/i2c-1"
#define CLOCK 72000000.0

// I2C Registers
#define REG_CHN  0x20
#define REG_PSC  0x40
#define REG_ARR  0x44
#define REG_PSC2 0x50
#define REG_ARR2 0x54

#define ADC_REG 0x17 // ADC channel register

int i2c_fd;

// Function to write to an I2C register
void i2c_write(int reg, int value) {
    unsigned char buffer[3];
    buffer[0] = reg;
    buffer[1] = (value >> 8) & 0xFF;
    buffer[2] = value & 0xFF;
    if (write(i2c_fd, buffer, 3) != 3) {
        perror("I2C write failed");
        exit(1);
    }
}

// Function to read from an I2C register
int i2c_read(int reg) {
    unsigned char buffer[2];
    if (write(i2c_fd, &reg, 1) != 1) {
        perror("I2C write (for read) failed");
        exit(1);
    }
    if (read(i2c_fd, buffer, 2) != 2) {
        perror("I2C read failed");
        exit(1);
    }
    return (buffer[0] << 8) | buffer[1];
}

// Set PWM frequency
void set_pwm_freq(float freq) {
    int psc, arr;
    double best_error = 1e9;
    
    for (int p = 1; p < 256; p++) {
        int a = (int)(CLOCK / (freq * p));
        double error = fabs(freq - (CLOCK / (p * a)));
        if (error < best_error) {
            best_error = error;
            psc = p;
            arr = a;
        }
    }

    i2c_write(REG_PSC, psc - 1);
    i2c_write(REG_ARR, arr);
}

// Set PWM duty cycle (0-100%)
void set_pwm_duty(int channel, float percent) {
    int pulse_width = (int)((percent / 100.0) * 65535);
    i2c_write(REG_CHN + channel, pulse_width);
}

int main() {
    // Open I2C device
    i2c_fd = open(I2C_DEV, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return 1;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, 0x14) < 0) {
        perror("Failed to set I2C slave address");
        return 1;
    }

    // Set PWM frequency to 50Hz
    set_pwm_freq(50);

    // Set duty cycle to 50%
    set_pwm_duty(12, 00.0);

    printf("PWM set to 50%% duty cycle\n");

    // Read ADC value from channel 0
    while(1){
        int adc_value = i2c_read(ADC_REG);
        printf("ADC Channel 0 Value: %d\n", adc_value);
        sleep(1);
    }

    close(i2c_fd);
    return 0;
}

