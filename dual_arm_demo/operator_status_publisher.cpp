#include "JAKAZuRobot.h"
#include "operator_status_ipc.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {
std::atomic<bool> running{true};
void handle_signal(int) { running.store(false); }
std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(now.tv_nsec);
}
}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <operator_ip> <status_socket> [duration_sec]\n";
        return 64;
    }
    const char* operator_ip = argv[1];
    const std::string status_socket = argv[2];
    const double duration_sec = argc == 4 ? std::stod(argv[3]) : 0.0;
    if (status_socket.size() >= sizeof(sockaddr_un::sun_path)) return 65;

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    const int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd < 0) return 66;
    sockaddr_un destination{};
    destination.sun_family = AF_UNIX;
    std::strncpy(destination.sun_path,
                 status_socket.c_str(),
                 sizeof(destination.sun_path) - 1);

    JAKAZuRobot robot;
    const int login_ret = robot.login_in(operator_ip, false);
    std::cout << "operator status login_ret=" << login_ret
              << " ip=" << operator_ip << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        return 1;
    }

    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t sequence = 0;
    while (running.load()) {
        if (duration_sec > 0.0 &&
            monotonic_ns() - started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) break;
        RobotStatus_simple status{};
        BOOL dragging = FALSE;
        const int status_ret = robot.get_robot_status_simple(&status);
        const int drag_ret = robot.is_in_drag_mode(&dragging);
        jaka_status_ipc::StatusPacket packet{};
        packet.magic = jaka_status_ipc::kMagic;
        packet.version = jaka_status_ipc::kVersion;
        packet.size = sizeof(packet);
        packet.sequence = sequence++;
        packet.monotonic_ns = monotonic_ns();
        packet.status_ret = status_ret;
        packet.drag_ret = drag_ret;
        packet.powered = status_ret == 0 && status.powered_on ? 1 : 0;
        packet.enabled = status_ret == 0 && status.enabled ? 1 : 0;
        packet.dragging = drag_ret == 0 && dragging ? 1 : 0;
        packet.valid = status_ret == 0 && drag_ret == 0 ? 1 : 0;
        sendto(socket_fd, &packet, sizeof(packet), 0,
               reinterpret_cast<const sockaddr*>(&destination),
               sizeof(destination));
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    const int logout_ret = robot.login_out();
    close(socket_fd);
    return logout_ret == 0 ? 0 : 2;
}
