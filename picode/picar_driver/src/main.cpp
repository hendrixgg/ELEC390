/**
 * @file main.cpp
 * @author Jacob Chisholm (https://jchisholm.github.io)
 * @brief PiCarX Driver Test File
 * @date 2025-02-26
 * @version 0.1
 *
 */
#include <stdio.h>
#include <iostream>
#include <unistd.h>
#include "pix_driver.hpp"


int main(int argc, char **argv){
    std::cout << "Hello World\n";
    printf("Hello World\n");
    PiX px = PiX();
    int power = 100;
    printf("Turning..\n");
    // px.set_turnAngle(30);
    // sleep(1);
    // px.set_turnAngle(-30);
    // sleep(1);
    // px.set_turnAngle(0);
    // px.pwm_set_pulse_width(12, 5000);
    // sleep(1);
    // px.pwm_set_pulse_width(12, 00);
    for(int i = 0; i < 7; i++){
        px.pwm_set_period(i, 4095);   // Example period
        px.pwm_set_frequency(i, 50);
    }
    // for(int i = 1; i < 14; i++){
    //     px.pwm_set_pulse_width(0, 10*i); // 50% duty cycle
    //     px.pwm_set_pulse_width(2, 10*i); // 50% duty cycle
    //     sleep(1);
    // }
    for(int i = 10; i < 12; i++){
        if(i == 12 || i == 13)
            continue;
        printf("%d\n", i);
        // px.pwm_set_prescaler(i, 10);  // Example prescaler
        px.pwm_set_pulse_width(i, 1000); // 50% duty cycle
        sleep(1);
        // px.pwm_set_prescaler(i, 10);  // Example prescaler
        // px.pwm_set_period(i, 1000);   // Example period
        px.pwm_set_pulse_width(i, 4095); // 50% duty cycle
        // sleep(1);
    }
    // for(int i = 0; i < 2; i++){
    //     printf("Driving Forwards\n");
    //     px.set_drivePower(power);
    //     sleep(1);
    //     printf("Driving Backwards\n");
    //     px.set_drivePower(-power);
    //     sleep(1);
    // }
}
