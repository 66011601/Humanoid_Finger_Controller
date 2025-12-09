#ifndef MOTOR_CONTROLS_HPP
#define MOTOR_CONTROLS_HPP

#include "can_bus.hpp"
#include <string>
#include <vector>
#include <cstdint>
#include <cmath> // For NAN

// Struct to hold decoded RMD feedback data
struct RMDFeedback {
    int msg_class = 0;
    int err_msg = 0;
    float pos = NAN;
    float current = NAN;
    float temp = NAN;
};

/**
 * @brief Base class for all motor types.
 * Mirrors the functionality of your 'MotorControl' base class in Python.
 */
class MotorControl {
protected:
    uint32_t id;
    CANBus *bus;
    std::string name;

public:
    MotorControl(uint32_t id, CANBus *bus, const std::string &name = "Motor");
    virtual ~MotorControl() = default;

    // Pure virtual functions (must be implemented by derived classes)
    virtual void set_state(int cmd) = 0;
    virtual std::vector<uint8_t> position_write(float pos_deg, float vel_rpm) = 0;
    virtual float position_read() = 0;
    virtual float read_feedback() = 0;
    virtual void move_and_monitor(float target_deg, float vel_rpm) = 0;

    // Getters
    uint32_t get_id() const { return id; }
    std::string get_name() const { return name; }
};

// --- Derived Motor Classes ---

class LKtech_Motor : public MotorControl {
public:
    LKtech_Motor(uint32_t id, CANBus *bus, const std::string &name = "LKtech_Motor");

    void set_state(int cmd) override;
    std::vector<uint8_t> position_write(float pos_deg, float vel_rpm) override;
    float position_read() override;
    float read_feedback() override;
    void move_and_monitor(float target_deg, float vel_rpm) override;
};

class RMD_Motor : public MotorControl {
public:
    RMD_Motor(uint32_t id, CANBus *bus, const std::string &name = "RMD_Motor");

    void set_state(int cmd) override;
    std::vector<uint8_t> position_write(float pos_deg, float vel_rpm) override;
    float position_read() override;
    float read_feedback() override;
    void move_and_monitor(float target_deg, float vel_rpm) override;
};

class RMD_BionicMotor : public MotorControl {
public:
    RMD_BionicMotor(uint32_t id, CANBus* bus, const std::string &name);

    // Correct override (matches base class)
    std::vector<uint8_t> position_write(float pos, float vel) override;

    // EXTRA extension (NOT override)
    std::vector<uint8_t> position_write(float pos, float vel, float cur);

    float position_read() override;
    float read_feedback() override;
    RMDFeedback read_feedback_struct(); // full feedback struct

    // Missing override (fix)
    void set_state(int cmd) override;

    void position_write_increment(float deg, float vel = 20, float cur = 5);
    
    // NEW: Function to move to an absolute position and monitor progress
    void position_write_absolute(float target_deg, float vel_rpm, float current_limit);
    
    void move_and_monitor(float target_deg, float vel_rpm) override;
};


#endif // MOTOR_CONTROLS_HPP
