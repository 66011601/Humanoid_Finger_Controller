#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <memory> // For std::unique_ptr
#include "motor_control.hpp"

// Define placeholder CAN IDs (Please check these IDs for your specific setup)
constexpr uint32_t LKTECH_CAN_ID = 0x141; // Common ID for LKtech motors
constexpr uint32_t RMD_BIONIC_CAN_ID = 0x01;  // Example ID for RMD Bionic (often low, e.g., 0x01)
constexpr uint32_t RMD_STANDARD_CAN_ID = 0x141; // Common ID for RMD Standard Motor 1

void usage() {
    std::cout << "\n======================================================\n";
    std::cout << "  Motor Control Test Application\n";
    std::cout << "======================================================\n";
    std::cout << "Select Motor Type to Test:\n";
    std::cout << "  1. MG (LKtech_Motor)\n";
    std::cout << "  2. BMD_Bionic (RMD_BionicMotor)\n";
    std::cout << "  3. BMD_Motor (RMD_Motor)\n";
    std::cout << "Enter selection (1, 2, or 3): ";
}

int main() {
    std::cout << "Initializing CAN bus (can0)..." << std::endl;
    CANBus bus("can0");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    std::unique_ptr<MotorControl> motor = nullptr;
    int selection = 0;
    
    // --- Motor Selection Loop ---
    while (!motor) {
        usage();
        std::cin >> selection;

        if (std::cin.fail()) {
            std::cin.clear(); // Clear error flags
            std::cin.ignore(10000, '\n'); // Discard bad input
            std::cout << "Invalid input. Please enter 1, 2, or 3.\n";
            continue;
        }

        switch (selection) {
            case 1:
                motor = std::make_unique<LKtech_Motor>(LKTECH_CAN_ID, &bus, "LKtech_MG6");
                std::cout << "\nSelected: LKtech MG6 (ID: 0x" << std::hex << LKTECH_CAN_ID << std::dec << ")\n";
                break;
            case 2:
                motor = std::make_unique<RMD_BionicMotor>(RMD_BIONIC_CAN_ID, &bus, "RMD_Bionic");
                std::cout << "\nSelected: RMD Bionic (ID: 0x" << std::hex << RMD_BIONIC_CAN_ID << std::dec << ")\n";
                break;
            case 3:
                motor = std::make_unique<RMD_Motor>(RMD_STANDARD_CAN_ID, &bus, "RMD_Standard");
                std::cout << "\nSelected: RMD Standard (ID: 0x" << std::hex << RMD_STANDARD_CAN_ID << std::dec << ")\n";
                break;
            default:
                std::cout << "Invalid selection. Please try again.\n";
        }
    }

    // --- Motor Enable and Main Loop ---
    
    // Command 0x81 (RMD motors) or 1 (LKtech motors) typically means enable/running
    motor->set_state(0x81); 
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Motor is ready. Type 'exit' to quit.\n" << std::endl;

    while (true) {
        std::string pos_input;
        std::cout << "Enter ABSOLUTE target position (degrees): ";
        std::cin >> pos_input;

        if (pos_input == "exit" || pos_input == "quit") break;
        if (std::cin.fail()) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        float target_deg = std::stof(pos_input);

        std::string vel_input;
        std::cout << "Enter velocity (RPM): ";
        std::cin >> vel_input;

        if (vel_input == "exit" || vel_input == "quit") break;
        if (std::cin.fail()) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        float vel_rpm = std::stof(vel_input);

        float current_limit = 5.0f; // Default current limit for Bionic motor

        std::cout << "\n--- Moving motor ---" << std::endl;

        // --- Execute Movement based on selection ---
        motor->move_and_monitor(target_deg, vel_rpm);
        
        std::cout << "--------------------\n" << std::endl;
    }

    // --- Cleanup ---
    // Command 0x80 (RMD) or 0 (LKtech) typically means motor stop/off
    motor->set_state(0x80);
    bus.shutdown();

    std::cout << "\nProgram terminated.\n" << std::endl;
    return 0;
}
