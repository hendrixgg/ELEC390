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
    printf("Turning.. 0\n");
    px.set_turnOffset(4);
    for(;;){
        printf("Line Sensors: %0.2f %0.2f %0.2f\n", px.get_lineSensor(0), px.get_lineSensor(1), px.get_lineSensor(2));
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // px.set_turnAngle(5);
    sleep(1);
    for(int i = 0; i < 200; i += 20){
        printf("Lifting... %d deg\n", i);
        px.set_liftAngle(i);
        sleep(1);
    }
    // for(int i = -30; i < 35; i+=5){
    //     printf("Turning... %d deg\n", i);
    //     px.set_turnAngle(i);
    //     px.set_cameraPan(i);
    //     px.set_cameraTilt(i);
    //     sleep(1);
    // }
    px.set_turnAngle(0);
    // for(int i = 0; i < 2; i++){
    //     printf("Driving Forwards\n");
    //     px.set_drivePower(power);
    //     sleep(1);
    //     printf("Driving Backwards\n");
    //     px.set_drivePower(-power);
    //     sleep(1);
    // }
}
