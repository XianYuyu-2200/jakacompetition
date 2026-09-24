#include "JAKAZuRobot.h"
#include "joint_sample_ipc.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <sys/socket.h>
#include <sys/un.h>
#include <poll.h>
#include <time.h>
#include <unistd.h>

namespace {
std::atomic<bool> running{true};
void stop(int) { running.store(false); }
std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(now.tv_nsec);
}
}

int main(int argc, char* argv[]) {
    if (argc < 4 || argc > 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <tracking_ip> <ipc_socket> START_J1 [duration_sec]\n";
        return 64;
    }
    if (std::string(argv[3]) != "START_J1") {
        std::cerr << "refusing to start without START_J1 token\n";
        return 65;
    }
    const char* tracking_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 5 ? std::stod(argv[4]) : 0.0;
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) return 66;
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const int fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (fd < 0) return 67;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path, socket_path.c_str(),
                 sizeof(address.sun_path) - 1);
    unlink(socket_path.c_str());
    if (bind(fd, reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
        close(fd);
        return 68;
    }
    JAKAZuRobot robot;
    const int login_ret = robot.login_in(tracking_ip, false);
    std::cout << "j1 follower login_ret=" << login_ret
              << " ip=" << tracking_ip << std::endl;
    if (login_ret != 0) { close(fd); return 1; }

    RobotStatus_simple status{};
    const int status_ret = robot.get_robot_status_simple(&status);
    if (status_ret != 0 || !status.powered_on || !status.enabled) {
        std::cerr << "tracking arm must already be powered and enabled; "
                  << "status_ret=" << status_ret
                  << " powered=" << status.powered_on
                  << " enabled=" << status.enabled << '\n';
        robot.login_out(); close(fd); return 2;
    }

    JointValue initial{};
    const int initial_ret = robot.get_actual_joint_position(&initial);
    if (initial_ret != 0) { robot.login_out(); close(fd); return 3; }
    double tracking_zero[6]{};
    for (int joint = 0; joint < 6; ++joint) tracking_zero[joint] = initial.jVal[joint];

    bool servo_enabled = false;
    bool have_operator_zero = false;
    double operator_zero[6]{};
    std::uint64_t last_packet_ns = 0;
    std::uint64_t last_sequence = 0;
    std::uint64_t samples = 0;
    std::uint64_t commands = 0;
    std::uint64_t max_latency_ns = 0;
    long double latency_sum_ns = 0.0;
    double max_error_rad = 0.0;
    long double error_sum_rad = 0.0;
    std::uint64_t error_samples = 0;
    const std::uint64_t launch_ns = monotonic_ns();
    std::uint64_t active_started_ns = 0;
    std::string stop_reason = "PROCESS_EXIT";
    while (running.load()) {
        if (active_started_ns != 0 && duration_sec > 0.0 &&
            monotonic_ns() - active_started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            stop_reason = "DURATION";
            break;
        }
        if (!have_operator_zero && monotonic_ns() - launch_ns >= 15'000'000'000ULL) {
            std::cerr << "operator readiness timeout, stopping\n";
            stop_reason = "OPERATOR_READINESS_TIMEOUT";
            break;
        }
        pollfd pfd{fd, POLLIN, 0};
        if (poll(&pfd, 1, 10) <= 0 || !(pfd.revents & POLLIN)) {
            if (last_packet_ns != 0 && monotonic_ns() - last_packet_ns >= 100'000'000ULL) {
                std::cerr << "DATA_TIMEOUT, stopping\n";
                stop_reason = "DATA_TIMEOUT";
                break;
            }
            continue;
        }
        jaka_ipc::JointSamplePacket packet{};
        ssize_t bytes = recv(fd, &packet, sizeof(packet), 0);
        std::uint64_t received_ns = monotonic_ns();
        while (true) {
            jaka_ipc::JointSamplePacket newer{};
            const ssize_t newer_bytes =
                recv(fd, &newer, sizeof(newer), MSG_DONTWAIT);
            if (newer_bytes != static_cast<ssize_t>(sizeof(newer))) break;
            packet = newer;
            bytes = newer_bytes;
            received_ns = monotonic_ns();
        }
        if (bytes != static_cast<ssize_t>(sizeof(packet)) ||
            !jaka_ipc::valid_packet(packet) || packet.sdk_code != 0) {
            std::cerr << "invalid operator packet, stopping\n";
            stop_reason = "INVALID_OPERATOR_PACKET";
            break;
        }
        const bool operator_ready =
            packet.operator_powered != 0 &&
            packet.operator_enabled != 0 &&
            packet.operator_dragging != 0;
        if (!operator_ready && !have_operator_zero) continue;
        if (!operator_ready) {
            std::cerr << "operator no longer ready, stopping\n";
            stop_reason = "OPERATOR_NOT_READY";
            break;
        }
        if (have_operator_zero && packet.sequence <= last_sequence) continue;
        last_sequence = packet.sequence;
        last_packet_ns = received_ns;
        if (!have_operator_zero) {
            for (int joint = 0; joint < 6; ++joint) operator_zero[joint] = packet.position[joint];
            have_operator_zero = true;
            std::cout << "operator zero captured sequence=" << packet.sequence << '\n';
            const int servo_ret = robot.servo_move_enable(TRUE);
            std::cout << "servo_move_enable_ret=" << servo_ret << std::endl;
            if (servo_ret != 0) {
                stop_reason = "SERVO_ENABLE_FAILED";
                break;
            }
            servo_enabled = true;
            active_started_ns = monotonic_ns();
        }
        JointValue command{};
        command.jVal[0] = tracking_zero[0] + (packet.position[0] - operator_zero[0]);
        for (int joint = 1; joint < 6; ++joint) command.jVal[joint] = tracking_zero[joint];
        const int command_ret = robot.servo_j(&command, MoveMode::ABS, 1);
        if (command_ret != 0) {
            std::cerr << "servo_j failed, stopping\n";
            stop_reason = "SERVO_J_FAILED";
            break;
        }
        ++commands;
        const std::uint64_t latency_ns = received_ns >= packet.monotonic_ns ?
            received_ns - packet.monotonic_ns : 0;
        if (latency_ns > max_latency_ns) max_latency_ns = latency_ns;
        latency_sum_ns += latency_ns;
        if (commands % 5 != 0) continue;
        JointValue actual{};
        const int actual_ret = robot.get_actual_joint_position(&actual);
        if (actual_ret != 0) {
            std::cerr << "tracking read failed, stopping\n";
            stop_reason = "TRACKING_READ_FAILED";
            break;
        }
        ++samples;
        const double error = std::abs(command.jVal[0] - actual.jVal[0]);
        if (error > max_error_rad) max_error_rad = error;
        error_sum_rad += error;
        ++error_samples;
        if (samples % 5 == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "seq=" << packet.sequence
                      << " operator_j1=" << packet.position[0]
                      << " target_j1=" << command.jVal[0]
                      << " actual_j1=" << actual.jVal[0]
                      << " error_deg=" << error * 180.0 / 3.14159265358979323846
                      << " latency_ms=" << static_cast<double>(latency_ns) / 1e6
                      << '\n';
        }
    }
    if (servo_enabled) {
        robot.motion_abort();
        robot.servo_move_enable(FALSE);
    }
    const int logout_ret = robot.login_out();
    close(fd);
    unlink(socket_path.c_str());
    std::cout << std::fixed << std::setprecision(6)
              << "j1 follow summary samples=" << samples
              << " commands=" << commands
              << " stop_reason=" << stop_reason
              << " active_duration_sec="
              << (active_started_ns == 0 ? 0.0 :
                  static_cast<double>(monotonic_ns() - active_started_ns) / 1e9)
              << " avg_latency_ms="
              << (commands == 0 ? 0.0 :
                  static_cast<double>(latency_sum_ns / commands / 1e6))
              << " max_latency_ms=" << static_cast<double>(max_latency_ns) / 1e6
              << " avg_error_deg="
              << (error_samples == 0 ? 0.0 :
                  static_cast<double>(error_sum_rad / error_samples) *
                      180.0 / 3.14159265358979323846)
              << " max_error_deg=" << max_error_rad * 180.0 / 3.14159265358979323846
              << " logout_ret=" << logout_ret << '\n';
    return commands > 0 && logout_ret == 0 ? 0 : 5;
}
