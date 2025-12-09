#include "can_bus.hpp"
#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>

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