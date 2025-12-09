#include "motor_control.hpp"
#include <iostream>
#include <algorithm>
#include <cmath>
#include <thread>
#include <chrono>

// ===============================================================
// Base MotorControl Implementation
// ===============================================================
MotorControl::MotorControl(uint32_t id, CANBus *bus, const std::string &name)
    : id(id), bus(bus), name(name) {}

// ===============================================================
// LKtech_Motor Implementation
// ===============================================================
LKtech_Motor::LKtech_Motor(uint32_t id, CANBus *bus, const std::string &name)
    : MotorControl(id, bus, name) {}

void LKtech_Motor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = static_cast<uint8_t>(cmd);
    bus->send_msg(id, payload);
}

std::vector<uint8_t> LKtech_Motor::position_write(float pos_deg, float vel_rpm) {
    std::vector<uint8_t> payload(8, 0);
    int32_t pos_int = (int32_t)(pos_deg * 3600.0f);
    uint16_t vel_raw = (uint16_t)(std::abs(vel_rpm) * 6.0f * 36.0f);
    // uint8_t vel_dir = (vel_rpm < 0) ? 1 : 0;
    float current_pos_raw = position_read();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    uint8_t vel_dir = (pos_deg - current_pos_raw < 0) ? 1 : 0;

    payload[0] = 0xA6;
    payload[1] = vel_dir;
    payload[2] = vel_raw & 0xFF;
    payload[3] = (vel_raw >> 8) & 0xFF;
    payload[4] = pos_int & 0xFF;
    payload[5] = (pos_int >> 8) & 0xFF;
    payload[6] = (pos_int >> 16) & 0xFF;
    payload[7] = (pos_int >> 24) & 0xFF;

    bus->send_msg(id, payload);
    return payload;
}

float LKtech_Motor::position_read() {
    std::vector<uint8_t> req(8, 0);
    req[0] = 0x94; // Read position command
    bus->send_msg(id, req);

    uint32_t r_id;
    std::vector<uint8_t> data;

    // Wait for response
    for (int i = 0; i < 20; i++) {
        if (!bus->read_msg(r_id, data)) continue;
        if (r_id != id) continue;
        if (data.size() < 8) continue;

        uint32_t raw = 
            (uint32_t)data[4] |
            ((uint32_t)data[5] << 8) |
            ((uint32_t)data[6] << 16) |
            ((uint32_t)data[7] << 24);

        float pos_deg = (float)raw / 3600.0f;
        return std::round(pos_deg * 100.0f) / 100.0f; 
    }
    return -1.0f; 
}

float LKtech_Motor::read_feedback() {
    return position_read();
}

void LKtech_Motor::move_and_monitor(float target_deg, float vel_rpm) 
{
    if (target_deg < 0 || target_deg > 360){ 
    	std::cerr << "\n[" << name << "] Warning: Please enter degree in range 0-360.\n"; 
    }
    else {
    	std::abs(target_deg) < 1.0f ? (target_deg >= 0 ? target_deg = 1.0f : target_deg = -1.0f) : target_deg = target_deg;
    	std::abs(target_deg) > 359.0f ? (target_deg > 0 ? target_deg = 359.0f : target_deg = -359.0f): target_deg = target_deg;
    
    	position_write(target_deg, vel_rpm);
    	std::cout << "\n[" << name << "] Moving to " << target_deg << " deg..." << std::endl;
    	int target_int = (int)std::round(target_deg) % 360;
    	if (target_int < 0) target_int += 360;

    	float tolerance = 1.0f;
    	const auto max_duration = std::chrono::seconds(15);
    	auto start_time = std::chrono::steady_clock::now();

    	for (;;) {
		    std::this_thread::sleep_for(std::chrono::milliseconds(20));
		    float current_pos_raw = position_read();

		    if (current_pos_raw < -0.9f) {
		        std::cout << "Feedback lost...\r";
		        std::cout.flush();
		        continue;
		    }

		    std::cout << "Current: " << current_pos_raw << " deg   \r";
		    std::cout.flush();

		    if (std::abs(current_pos_raw - target_int) <= tolerance) {
		        std::cout << "\nReached destination." << std::endl;
		        break;
		    }
		    
		    if (std::chrono::steady_clock::now() - start_time > max_duration) {
		        std::cerr << "\n[" << name << "] Warning: Timeout waiting for target position.\n";
		        break;
		    }
    	}
    }
}


// ===============================================================
// RMD_Motor Implementation
// ===============================================================
RMD_Motor::RMD_Motor(uint32_t id, CANBus *bus, const std::string &name)
    : MotorControl(id, bus, name) {}

void RMD_Motor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = (uint8_t)cmd; 
    bus->send_msg(id, payload);
    std::cout << "[" << name << "] Sent set state command 0x" << std::hex << cmd << std::dec << std::endl;
}

std::vector<uint8_t> RMD_Motor::position_write(float pos, float vel) {
    std::vector<uint8_t> payload(8, 0);
    int32_t p = (int32_t)(pos * 100.0f);
    uint16_t vel_raw = (uint16_t)(std::abs(vel) * 6.0f);
    uint8_t vel_dir = (vel < 0.0f) ? 0x00 : 0x01;
    
    payload[0] = 0xA4; 
    payload[1] = vel_dir; 
    payload[2] = vel_raw & 0xFF;
    payload[3] = (vel_raw >> 8) & 0xFF;
    payload[4] = p & 0xFF;
    payload[5] = (p >> 8) & 0xFF;
    payload[6] = (p >> 16) & 0xFF;
    payload[7] = (p >> 24) & 0xFF;

    bus->send_msg(id, payload);
    return payload;
}

float RMD_Motor::position_read() {
    std::vector<uint8_t> req(8, 0);
    req[0] = 0x92; // Read multi-turn position
    bus->send_msg(id, req); 

    uint32_t r_id;
    std::vector<uint8_t> data;
    
    for (int i = 0; i < 20; ++i) {
        if (!bus->read_msg(r_id, data)) {
             std::this_thread::sleep_for(std::chrono::milliseconds(5));
             continue;
        }
        
        // Response check 
        if (r_id != 0x241) continue; // The original C++ used a fixed ID 0x241
        if (data.size() < 8 || data[0] != 0x92) continue;

        int32_t raw_pos = (data[4] | (data[5] << 8) | (data[6] << 16) | (data[7] << 24)); 
        
        return (float)raw_pos / 100.0f;
    }
    return 0.0f; 
}

float RMD_Motor::read_feedback() {
    return position_read();
}

void RMD_Motor::move_and_monitor(float target_deg, float vel_rpm) {
    std::cout << "\n[" << name << "] Moving to absolute target: " << target_deg << " deg (Speed: " << vel_rpm << " RPM)..." << std::endl;
    
    const float tolerance = 1.0f; 
    const auto sleep_interval = std::chrono::milliseconds(50);
    const auto max_duration = std::chrono::seconds(10);
    auto start_time = std::chrono::steady_clock::now();
    
    float normalized_target = std::round(target_deg * 100.0f) / 100.0f;

    while (true) { 
        position_write(target_deg, vel_rpm); 

        std::this_thread::sleep_for(sleep_interval);
        
        float current_pos = position_read();
        
        std::cout << "[" << name << "] Current: " << current_pos << " deg | Target: " << target_deg << " deg   \r";
        std::cout.flush();
        
        if (std::abs(current_pos - normalized_target) <= tolerance) {
            std::cout << "\n[" << name << "] Target reached." << std::endl;
            break;
        }
        
        if (std::chrono::steady_clock::now() - start_time > max_duration) {
            std::cerr << "\n[" << name << "] Warning: Timeout waiting for target position.\n";
            break;
        }
    }
}


// ---------------------------
// RMD_BionicMotor Implementation (matches Python bit-packed 64-bit protocol)
// ---------------------------
RMD_BionicMotor::RMD_BionicMotor(uint32_t id_, CANBus *bus_, const std::string &name_)
    : MotorControl(id_, bus_, name_) {}

// set_state: send simple 8-byte payload with first byte as command
void RMD_BionicMotor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = static_cast<uint8_t>(cmd & 0xFF);
    bus->send_msg(id, payload);
}

// Helper: float -> IEEE754 uint32
static uint32_t float_to_uint32(float f) {
    union { float f; uint32_t u; } conv;
    conv.f = f;
    return conv.u;
}

// Helper: read 64-bit frame bytes -> uint64_t (big-endian assembled)
static uint64_t bytes_to_uint64_be(const std::vector<uint8_t> &data) {
    uint64_t v = 0;
    for (size_t i = 0; i < 8 && i < data.size(); ++i) {
        v = (v << 8) | static_cast<uint64_t>(data[i]);
    }
    return v;
}

// position_write(pos, vel) -> uses default current = 5.0
std::vector<uint8_t> RMD_BionicMotor::position_write(float pos, float vel) {
    return position_write(pos, vel, 5.0f);
}

// position_write(pos, vel, cur) -> builds 64-bit packed frame
std::vector<uint8_t> RMD_BionicMotor::position_write(float pos, float vel, float cur) {
    std::vector<uint8_t> payload(8, 0);

    // velocity * 10 -> 15 bits
    uint32_t vel_raw = static_cast<uint32_t>(std::round(std::abs(vel) * 10.0f)) & 0x7FFF; // 15 bits
    // current * 10 -> 12 bits
    uint32_t cur_raw = static_cast<uint32_t>(std::round(std::abs(cur) * 10.0f)) & 0x0FFF; // 12 bits

    uint32_t pos_bits = float_to_uint32(pos); // IEEE754 bits

    // Build 64-bit frame according to Python layout:
    // bits 63..61 : 3-bit header (0b001)
    // bits 60..29 : 32-bit IEEE754 pos
    // bits 28..14 : 15-bit vel
    // bits 13..2  : 12-bit cur
    // bits 1..0   : 2-bit footer 0b10

    uint64_t frame = 0;
    frame |= (uint64_t)0x1ULL << 61; // header 0b001 placed at bits 63..61 (value 1 << 61)
    frame |= (uint64_t)pos_bits << 29;
    frame |= (uint64_t)vel_raw << 14;
    frame |= (uint64_t)cur_raw << 2;
    frame |= (uint64_t)0x2ULL; // footer 0b10

    // Split into bytes (big-endian order)
    for (int i = 0; i < 8; ++i) {
        payload[i] = static_cast<uint8_t>((frame >> (56 - i * 8)) & 0xFF);
    }

    bus->send_msg(id, payload);
    return payload;
}

// position_read(): send read request and parse position (float)
float RMD_BionicMotor::position_read() {
    // Send read request (0x0E 0x00 0x00 0x01)
    std::vector<uint8_t> req(8, 0);
    req[0] = 0x0E;
    req[3] = 0x01;
    bus->send_msg(id, req);

    uint32_t rid;
    std::vector<uint8_t> data;

    for (int i = 0; i < 20; ++i) {
        if (!bus->read_msg(rid, data)) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        if (rid != id) continue;
        if (data.size() < 8) continue;

        uint64_t frame = bytes_to_uint64_be(data);

        // POS = bits [8..39] (Python msg_bin) -> frame bits [55..24] (Big-Endian)
        uint32_t pos_bits = (uint32_t)((frame >> 24) & 0xFFFFFFFFULL);

        union { uint32_t u; float f; } conv;
        conv.u = pos_bits;
        // Match Python's rounding (round to 1 decimal place)
        return std::round(conv.f * 10.0f) / 10.0f; 
    }

    return -100000.0f;
}

// read_feedback(): returns only position (float)
float RMD_BionicMotor::read_feedback() {
    RMDFeedback fb = read_feedback_struct();
    return fb.pos;
}

// read_feedback_struct(): full decoding of a single 8-byte response frame
RMDFeedback RMD_BionicMotor::read_feedback_struct() {
    RMDFeedback out;
    out.msg_class = -1;
    out.err_msg = -1;
    out.pos = -100000.0f;
    out.current = 0.0f;
    out.temp = NAN; // Set to NaN initially

    uint32_t rid;
    std::vector<uint8_t> data;
    if (!bus->read_msg(rid, data)) return out;
    if (rid != id) return out;
    if (data.size() < 8) return out;

    uint64_t frame = bytes_to_uint64_be(data);

    // --- Parse fields to match Python logic ---

    // 1. msg_class: bits 63..61 (Python msg_bin[:3])
    int msg_class = static_cast<int>((frame >> 61) & 0x7ULL);      

    // 2. err_msg: bits 58..54 (Python msg_bin[3:8])
    int err_msg = static_cast<int>((frame >> 56) & 0x1FUL);

    // 3. pos: bits 55..24 (Python msg_bin[8:40])
    uint32_t pos_bits = static_cast<uint32_t>((frame >> 24) & 0xFFFFFFFFULL);
    union { uint32_t u; float f; } conv;
    conv.u = pos_bits;
    float pos_f = std::round(conv.f * 10.0f) / 10.0f; // Round to 1 decimal place

    // 4. current: bits 23..8 (Python msg_bin[40:56], then / 100)
    uint32_t current_raw_16bit = static_cast<uint32_t>((frame >> 8) & 0xFFFFULL);
    float current_f = std::round((static_cast<float>(current_raw_16bit) / 100.0f) * 100.0f) / 100.0f; // Round to 2 decimal places

    // 5. temp: bits 7..0 (Python msg_bin[56:], then scale)
    uint32_t temp_raw = static_cast<uint32_t>(frame & 0xFFULL); 
    float temp_f = std::round(((static_cast<float>(temp_raw) - 50.0f) / 2.0f) * 10.0f) / 10.0f; // Round to 1 decimal place

    out.msg_class = msg_class;
    out.err_msg = err_msg;
    out.pos = pos_f;
    out.current = current_f;
    out.temp = temp_f;
    return out;
}

// position_write_increment: behavioral loop until target reached
void RMD_BionicMotor::position_write_increment(float deg, float vel, float cur) {
    // Read current position and cast/round to integer target, matching Python
    int current = static_cast<int>(std::round(position_read()));
    if (current < -50000) {
        std::cerr << "[RMD_BionicMotor] position_write_increment: failed to read current position\n";
        return;
    }

    int target = current + static_cast<int>(std::round(deg));
    const float tolerance = 0.1f;
    const auto sleep_interval = std::chrono::milliseconds(200);
    const auto max_duration = std::chrono::seconds(15);

    auto start = std::chrono::steady_clock::now();

    while (true) {
        // send command (using float target)
        position_write((float)target, vel, cur);

        std::this_thread::sleep_for(sleep_interval);

        // read feedback (structured) and check pos
        RMDFeedback fb = read_feedback_struct();
        if (fb.msg_class != -1) {
            std::cout << "[RMD_BionicMotor] current: " << fb.pos << " target: " << target << std::endl;
            // Check if the rounded current position matches the integer target
            if (std::fabs(std::round(fb.pos) - target) <= 0.5f) { // Use 0.5f to check against integer
                std::cout << "[RMD_BionicMotor] target reached." << std::endl;
                break;
            }
        } else {
            std::cerr << "[RMD_BionicMotor] no feedback, retrying..." << std::endl;
        }

        if (std::chrono::steady_clock::now() - start > max_duration) {
            std::cerr << "[RMD_BionicMotor] position_write_increment: timeout\n";
            break;
        }
    }
}

// NEW FUNCTION: position_write_absolute: behavioral loop until absolute target reached
void RMD_BionicMotor::position_write_absolute(float target_deg, float vel_rpm, float current_limit) {
    std::cout << "\n[RMD_BionicMotor] Moving to absolute target: " << target_deg << " deg..." << std::endl;
    
    // Constants for the loop
    const float tolerance = 1.0f; 
    const auto sleep_interval = std::chrono::milliseconds(200);
    const auto max_duration = std::chrono::seconds(15);
    auto start_time = std::chrono::steady_clock::now();

    while (true) { 
        // 1. Re-send the absolute command continuously
        position_write(target_deg, vel_rpm, current_limit); 

        std::this_thread::sleep_for(sleep_interval);
        
        // 2. Read feedback
        RMDFeedback fb = read_feedback_struct();
        
        // 3. Check for read errors
        if (fb.pos < -50000.0f) {
            std::cerr << "[RMD_BionicMotor] Warning: Failed to read feedback.\n";
            continue;
        }
        
        // 4. Print status on a single line
        std::cout << "[RMD_BionicMotor] Current: " << fb.pos << " deg | Target: " << target_deg << " deg   \r";
        std::cout.flush();
        
        // 5. Check if target is reached
        if (std::abs(fb.pos - target_deg) <= tolerance) {
            std::cout << "\n[RMD_BionicMotor] Target reached." << std::endl;
            break;
        }
        
        // 6. Check for timeout
        if (std::chrono::steady_clock::now() - start_time > max_duration) {
            std::cerr << "\n[RMD_BionicMotor] Warning: Timeout waiting for target position.\n";
            break;
        }
    }
}

void RMD_BionicMotor::move_and_monitor(float target_deg, float vel_rpm) {
    // This motor uses its specialized function, so we call it here with a default current limit.
    // NOTE: In the actual application, you would use this to implement monitoring for this motor type.
    std::cerr << "[" << name << "] Bionic motor is using its absolute write, not 'move_and_monitor'.\n";
    position_write_absolute(target_deg, vel_rpm, 5.0f); // Default 5A current limit
}

