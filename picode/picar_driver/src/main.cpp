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
    for(int i = 0; i < 2; i++){
        px.set_drivePower(20);
        sleep(1);
        px.set_drivePower(-20);
        sleep(1);
    }
}
