#ifndef CAN_BUS_HPP
#define CAN_BUS_HPP

#include <string>
#include <vector>
#include <cstdint>

class CANBus {
private:
    int socket_fd = -1;

public:
    /**
     * @brief Initializes the CAN socket for a given interface (e.g., "can0").
     */
    CANBus(const std::string &interface);

    /**
     * @brief Sends a CAN message.
     * @param id The arbitration ID.
     * @param data The 8-byte payload.
     * @return True on success.
     */
    bool send_msg(uint32_t id, const std::vector<uint8_t> &data);

    /**
     * @brief Reads a CAN message (blocking).
     * @param id Reference to store the received arbitration ID.
     * @param data Reference to store the received payload.
     * @return True on success.
     */
    bool read_msg(uint32_t &id, std::vector<uint8_t> &data);

    /**
     * @brief Closes the CAN socket.
     */
    void shutdown();

    // Prevent copy/move
    CANBus(const CANBus&) = delete;
    CANBus& operator=(const CANBus&) = delete;
};

#endif // CAN_BUS_HPP