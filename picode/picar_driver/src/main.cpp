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
    for(int i = 0; i < 2; i++){
        printf("Driving Forwards\n");
        px.set_drivePower(power);
        sleep(1);
        printf("Driving Backwards\n");
        px.set_drivePower(-power);
        sleep(1);
    }
}
