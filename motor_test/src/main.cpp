#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include "motor_control.hpp"

int main() {

    std::cout << "Initializing CAN bus (can0)..." << std::endl;

    CANBus bus("can0");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    // Create motor instance
    LKtech_Motor MG6(0x141, &bus, "MG6");

    MG6.set_state(1);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "MG6 motor is ready." << std::endl;
    std::cout << "Type 'exit' to quit." << std::endl;

    while (true) {
        // ---------------------------------------
        // Ask user for target position
        // ---------------------------------------
        std::string pos_input;
        std::cout << "\nEnter target position (degrees): ";
        std::cin >> pos_input;

        if (pos_input == "exit" || pos_input == "quit") 
            break;

        float target_deg = std::stof(pos_input);

        // ---------------------------------------
        // Ask user for velocity (RPM)
        // ---------------------------------------
        std::string vel_input;
        std::cout << "Enter velocity (RPM): ";
        std::cin >> vel_input;

        if (vel_input == "exit" || vel_input == "quit") 
            break;

        float vel_rpm = std::stof(vel_input);

        // ---------------------------------------
        // Send movement command
        // ---------------------------------------
        MG6.move_and_monitor(target_deg, vel_rpm);
    }

    MG6.set_state(0);
    bus.shutdown();

    std::cout << "Program terminated." << std::endl;
    return 0;
}

