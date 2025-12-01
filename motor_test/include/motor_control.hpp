#pragma once
#include <vector>
#include <string>
#include <cstdint>

// ==========================
// CAN BUS WRAPPER (SocketCAN)
// ==========================
class CANBus {
public:
    CANBus(const std::string &interface);
    bool send_msg(uint32_t id, const std::vector<uint8_t> &data);
    bool read_msg(uint32_t &id, std::vector<uint8_t> &data);
    void shutdown();
private:
    int socket_fd;
};

// ==========================
// Base Abstract Motor Class
// ==========================
class MotorControl {
public:
    MotorControl(uint32_t id, CANBus* bus, const std::string &name);

    virtual void set_state(int state_code) = 0;
    virtual std::vector<uint8_t> position_write(float pos, float vel) = 0;
    virtual float position_read() = 0;
    virtual float read_feedback() = 0;

protected:
    uint32_t id;
    CANBus* bus;
    std::string name;
};

// ==========================
// LKtech MG6 Motor
// ==========================
class LKtech_Motor : public MotorControl {
public:
    LKtech_Motor(uint32_t id, CANBus* bus, const std::string &name);

    void set_state(int cmd) override;
    std::vector<uint8_t> position_write(float pos, float vel) override;
    float position_read() override;
    float read_feedback() override;
    
    void move_and_monitor(float target_deg, float vel_rpm);
};

// ==========================
// RMD X8 Bionic Motor
// ==========================
class RMD_BionicMotor : public MotorControl {
public:
    RMD_BionicMotor(uint32_t id, CANBus* bus, const std::string &name);

    // Correct override (matches base class)
    std::vector<uint8_t> position_write(float pos, float vel) override;

    // EXTRA extension (NOT override)
    std::vector<uint8_t> position_write(float pos, float vel, float cur);

    float position_read() override;
    float read_feedback() override;

    // Missing override (fix)
    void set_state(int cmd) override;

    void position_write_increment(float deg, float vel = 20, float cur = 5);
};

// ==========================
// RMD X8 Standard Motor
// ==========================
class RMD_Motor : public MotorControl {
public:
    RMD_Motor(uint32_t id, CANBus* bus, const std::string &name);

    void set_state(int cmd) override;
    std::vector<uint8_t> position_write(float pos, float vel) override;
    float position_read() override;
    float read_feedback() override;
};

