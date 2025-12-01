#include "motor_control.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <cmath>
#include <chrono>
#include <thread>

// ================================================================
// CANBus Implementation
// ================================================================
CANBus::CANBus(const std::string &interface) {
    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        perror("Socket creation failed");
        return;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ);
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("CAN socket bind failed");
        close(socket_fd);
        socket_fd = -1;
    }
}

bool CANBus::send_msg(uint32_t id, const std::vector<uint8_t> &data) {
    if (socket_fd < 0) return false;

    struct can_frame frame {};
    frame.can_id = id;
    frame.can_dlc = data.size();
    for (int i = 0; i < frame.can_dlc; i++)
        frame.data[i] = data[i];

    int nbytes = write(socket_fd, &frame, sizeof(frame));
    return (nbytes == sizeof(frame));
}

bool CANBus::read_msg(uint32_t &id, std::vector<uint8_t> &data) {
    if (socket_fd < 0) return false;

    struct can_frame frame {};
    int nbytes = read(socket_fd, &frame, sizeof(frame));
    if (nbytes < 0) return false;

    id = frame.can_id;
    data.resize(frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; i++)
        data[i] = frame.data[i];

    return true;
}

void CANBus::shutdown() {
    if (socket_fd >= 0) close(socket_fd);
}

// ================================================================
// MotorControl Base Class
// ================================================================
MotorControl::MotorControl(uint32_t id, CANBus *bus, const std::string &name)
    : id(id), bus(bus), name(name) {}

// ================================================================
// LKtech MG6 Motor Implementation
// ================================================================
LKtech_Motor::LKtech_Motor(uint32_t id, CANBus *bus, const std::string &name)
    : MotorControl(id, bus, name) {}

void LKtech_Motor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = static_cast<uint8_t>(cmd);
    bus->send_msg(id, payload);
}

std::vector<uint8_t> LKtech_Motor::position_write(float pos_deg, float vel_rpm) {
    std::vector<uint8_t> payload(8, 0);

    // Convert pos from degrees to MG6 internal units
    // pos_int = degrees * 3600   (as Python code)
    int32_t pos_int = (int32_t)(pos_deg * 3600.0f);

    // Convert velocity RPM -> MG6 units
    // vel_raw = abs(vel) * 6 * 36  (as Python code)
    uint16_t vel_raw = (uint16_t)(std::abs(vel_rpm) * 6.0f * 36.0f);

    // Direction (0 = CW, 1 = CCW)
    uint8_t vel_dir = (vel_rpm < 0) ? 0 : 1;

    payload[0] = 0xA6;
    payload[1] = vel_dir;
    payload[2] = vel_raw & 0xFF;
    payload[3] = (vel_raw >> 8) & 0xFF;

    // 32-bit little-endian position
    payload[4] = pos_int & 0xFF;
    payload[5] = (pos_int >> 8) & 0xFF;
    payload[6] = (pos_int >> 16) & 0xFF;
    payload[7] = (pos_int >> 24) & 0xFF;

    bus->send_msg(id, payload);
    return payload;
}


float LKtech_Motor::position_read() {
    // 1) Send the request command (0x94)
    std::vector<uint8_t> req(8, 0);
    req[0] = 0x94;
    bus->send_msg(id, req);

    // 2) Read until we get a frame from this motor
    uint32_t r_id;
    std::vector<uint8_t> data;

    for (int i = 0; i < 20; i++) {
        if (!bus->read_msg(r_id, data)) continue;

        // Must match motor ID
        if (r_id != id) continue;

        // Must have at least 8 bytes
        if (data.size() < 8) continue;

        // 3) Decode little-endian 32-bit position
        uint32_t raw = 
            (uint32_t)data[4] |
            ((uint32_t)data[5] << 8) |
            ((uint32_t)data[6] << 16) |
            ((uint32_t)data[7] << 24);

        // 4) Convert ticks → degrees
        float pos_deg = (float)raw / 3600.0f;
        float final_deg = (float)pos_deg * 90.0 / 298262.0 ;

        return pos_deg  ;
    }

    return -1.0f; // failed
}

float LKtech_Motor::read_feedback() {
    return position_read();
}

void LKtech_Motor::move_and_monitor(float target_deg, float vel_rpm) 
{
    // ---------------------------
    // 1. Send movement command
    // ---------------------------
    position_write(target_deg, vel_rpm);

    std::cout << "\nMoving to " << target_deg << " deg..." << std::endl;

    // Normalize angle to 0–359
    int current_deg = (int)target_deg % 360;
    if (current_deg < 0) current_deg += 360;

    // ---------------------------
    // 2. Real-time monitoring loop
    // ---------------------------
    float tolerance = 1.0f;
    float current_pos = 0.0f;

    for (;;) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));

        current_pos = position_read();

        if (current_pos < -30000) {    // failed read
            std::cout << "Feedback lost...\r";
            std::cout.flush();
            continue;
        }

        // Print real-time feedback on one line
        std::cout << "Current: " << current_pos << " deg   \r";
        std::cout.flush();

        // Check if motor has reached target
        if (std::abs(current_pos - target_deg) <= tolerance) {
            std::cout << "\nReached destination." << std::endl;
            break;
        }
    }
}

// ================================================================
// RMD Bionic Motor Implementation (X8 90-degree)
// ================================================================
RMD_BionicMotor::RMD_BionicMotor(uint32_t id, CANBus *bus, const std::string &name)
    : MotorControl(id, bus, name) {}

void RMD_BionicMotor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = (uint8_t)cmd;
    bus->send_msg(id, payload);
}

std::vector<uint8_t> RMD_BionicMotor::position_write(float pos, float vel) {
    std::vector<uint8_t> payload(8, 0);

    int32_t p = (int32_t)(pos * 100);
    int16_t v = (int16_t)(vel * 100);

    payload[0] = 0xA4;  // command
    payload[1] = p & 0xFF;
    payload[2] = (p >> 8) & 0xFF;
    payload[3] = (p >> 16) & 0xFF;
    payload[4] = (p >> 24) & 0xFF;
    payload[5] = v & 0xFF;
    payload[6] = (v >> 8) & 0xFF;

    bus->send_msg(id, payload);
    return payload;
}

std::vector<uint8_t> RMD_BionicMotor::position_write(float pos, float vel, float cur) {
    std::vector<uint8_t> payload(8, 0);

    int32_t p = (int32_t)(pos * 100);
    int16_t v = (int16_t)(vel * 100);
    int16_t c = (int16_t)(cur * 100);

    payload[0] = 0xA5;  // advanced position command
    payload[1] = p & 0xFF;
    payload[2] = (p >> 8) & 0xFF;
    payload[3] = (p >> 16) & 0xFF;
    payload[4] = (p >> 24) & 0xFF;

    payload[5] = v & 0xFF;
    payload[6] = (v >> 8) & 0xFF;

    payload[7] = c & 0xFF;  // current

    bus->send_msg(id, payload);
    return payload;
}

float RMD_BionicMotor::position_read() {
    uint32_t r_id;
    std::vector<uint8_t> data;

    if (bus->read_msg(r_id, data)) {
        if (data.size() >= 4) {
            int32_t raw = (data[1] | (data[2] << 8));
            return raw * 0.01f;
        }
    }
    return 0.0f;
}

float RMD_BionicMotor::read_feedback() {
    return position_read();
}

void RMD_BionicMotor::position_write_increment(float deg, float vel, float cur) {
    position_write(deg, vel, cur);
}

// ================================================================
// RMD Standard Motor Implementation
// ================================================================
RMD_Motor::RMD_Motor(uint32_t id, CANBus *bus, const std::string &name)
    : MotorControl(id, bus, name) {}

void RMD_Motor::set_state(int cmd) {
    std::vector<uint8_t> payload(8, 0);
    payload[0] = (uint8_t)cmd;
    bus->send_msg(id, payload);
}

std::vector<uint8_t> RMD_Motor::position_write(float pos, float vel) {
    std::vector<uint8_t> payload(8, 0);
    int32_t p = (int32_t)(pos * 100);

    payload[0] = 0xA4;
    payload[1] = p & 0xFF;
    payload[2] = (p >> 8) & 0xFF;
    payload[3] = (p >> 16) & 0xFF;
    payload[4] = (p >> 24) & 0xFF;

    bus->send_msg(id, payload);
    return payload;
}

float RMD_Motor::position_read() {
    uint32_t r_id;
    std::vector<uint8_t> data;

    if (bus->read_msg(r_id, data)) {
        if (data.size() >= 4) {
            int32_t raw = (data[1] | (data[2] << 8));
            return raw * 0.01f;
        }
    }
    return 0.0f;
}

float RMD_Motor::read_feedback() {
    return position_read();
}

