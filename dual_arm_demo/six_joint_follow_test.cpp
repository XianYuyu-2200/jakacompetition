#include "JAKAZuRobot.h"
#include "joint_sample_ipc.hpp"
#include "six_joint_mapping.hpp"

#include <atomic>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>

#include <poll.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {

constexpr double kRadToDeg = 180.0 / 3.14159265358979323846;
std::atomic<bool> running{true};

void stop(int) {
    running.store(false);
}

std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(now.tv_nsec);
}

void print_joint_array(const double values[6], double scale = 1.0) {
    std::cout << '[';
    for (int joint = 0; joint < 6; ++joint) {
        if (joint != 0) std::cout << ", ";
        std::cout << values[joint] * scale;
    }
    std::cout << ']';
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 4 || argc > 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <tracking_ip> <ipc_socket> START_SIX_JOINTS"
                     " [duration_sec]\n";
        return 64;
    }
    if (std::string(argv[3]) != "START_SIX_JOINTS") {
        std::cerr << "refusing to start without START_SIX_JOINTS token\n";
        return 65;
    }

    const char* tracking_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 5 ? std::stod(argv[4]) : 0.0;
    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) return 66;

    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);

    const int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd < 0) return 67;
    sockaddr_un address{};
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path,
                 socket_path.c_str(),
                 sizeof(address.sun_path) - 1);
    unlink(socket_path.c_str());
    if (bind(socket_fd,
             reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
        close(socket_fd);
        return 68;
    }

    JAKAZuRobot robot;
    const int login_ret = robot.login_in(tracking_ip, false);
    std::cout << "six-joint follower login_ret=" << login_ret
              << " ip=" << tracking_ip << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        unlink(socket_path.c_str());
        return 1;
    }

    RobotStatus_simple tracking_status{};
    const int status_ret = robot.get_robot_status_simple(&tracking_status);
    if (status_ret != 0 || !tracking_status.powered_on ||
        !tracking_status.enabled) {
        std::cerr << "tracking arm must already be powered and enabled; "
                  << "status_ret=" << status_ret
                  << " powered=" << tracking_status.powered_on
                  << " enabled=" << tracking_status.enabled << '\n';
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 2;
    }

    JointValue initial{};
    const int initial_ret = robot.get_actual_joint_position(&initial);
    if (initial_ret != 0) {
        robot.login_out();
        close(socket_fd);
        unlink(socket_path.c_str());
        return 3;
    }

    double tracking_zero[6]{};
    double operator_zero[6]{};
    for (int joint = 0; joint < 6; ++joint) {
        tracking_zero[joint] = initial.jVal[joint];
    }

    bool have_operator_zero = false;
    bool servo_enabled = false;
    std::uint64_t last_packet_ns = 0;
    std::uint64_t last_sequence = 0;
    std::uint64_t commands = 0;
    std::uint64_t state_samples = 0;
    std::uint64_t max_latency_ns = 0;
    long double latency_sum_ns = 0.0;
    long double error_sum_rad[6]{};
    double max_error_rad[6]{};
    const std::uint64_t launch_ns = monotonic_ns();
    std::uint64_t active_started_ns = 0;
    std::uint64_t active_ended_ns = 0;
    std::string stop_reason="PROCESS_EXIT";

    while (running.load()) {
        const std::uint64_t loop_now_ns = monotonic_ns();
        if (active_started_ns != 0 && duration_sec > 0.0 &&
            loop_now_ns - active_started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            stop_reason = "DURATION";
            break;
        }
        if (!have_operator_zero &&
            loop_now_ns - launch_ns >= 15'000'000'000ULL) {
            stop_reason = "OPERATOR_READINESS_TIMEOUT";
            break;
        }

        pollfd descriptor{socket_fd, POLLIN, 0};
        if (poll(&descriptor, 1, 10) <= 0 ||
            !(descriptor.revents & POLLIN)) {
            if (last_packet_ns != 0 &&
                monotonic_ns() - last_packet_ns >= 100'000'000ULL) {
                stop_reason = "DATA_TIMEOUT";
                break;
            }
            continue;
        }

        jaka_ipc::JointSamplePacket packet{};
        ssize_t bytes = recv(socket_fd, &packet, sizeof(packet), 0);
        std::uint64_t received_ns = monotonic_ns();
        while (true) {
            jaka_ipc::JointSamplePacket newer{};
            const ssize_t newer_bytes =
                recv(socket_fd, &newer, sizeof(newer), MSG_DONTWAIT);
            if (newer_bytes != static_cast<ssize_t>(sizeof(newer))) break;
            packet = newer;
            bytes = newer_bytes;
            received_ns = monotonic_ns();
        }

        if (bytes != static_cast<ssize_t>(sizeof(packet)) ||
            !jaka_ipc::valid_packet(packet) || packet.sdk_code != 0) {
            stop_reason = "INVALID_OPERATOR_PACKET";
            break;
        }

        const bool operator_ready =
            packet.operator_powered != 0 &&
            packet.operator_enabled != 0 &&
            packet.operator_dragging != 0;
        if (!operator_ready && !have_operator_zero) continue;
        if (!operator_ready) {
            stop_reason = "OPERATOR_NOT_READY";
            break;
        }
        if (have_operator_zero && packet.sequence <= last_sequence) continue;

        last_sequence = packet.sequence;
        last_packet_ns = received_ns;
        if (!have_operator_zero) {
            for (int joint = 0; joint < 6; ++joint) {
                operator_zero[joint] = packet.position[joint];
            }
            have_operator_zero = true;
            std::cout << "operator zero captured sequence="
                      << packet.sequence << '\n';
            const int servo_enable_ret = robot.servo_move_enable(TRUE);
            std::cout << "servo_move_enable_ret="
                      << servo_enable_ret << std::endl;
            if (servo_enable_ret != 0) {
                stop_reason = "SERVO_ENABLE_FAILED";
                break;
            }
            servo_enabled = true;
            active_started_ns = monotonic_ns();
        }

        JointValue command{};
        double operator_position[6]{};
        double target_position[6]{};
        for (int joint = 0; joint < 6; ++joint) {
            operator_position[joint] = packet.position[joint];
        }
        const auto mapped_target = jaka_six_joint::map_relative(
            operator_position, operator_zero, tracking_zero);
        for (int joint = 0; joint < 6; ++joint) {
            command.jVal[joint] = mapped_target[joint];
            target_position[joint] = mapped_target[joint];
        }

        const int command_ret =
            robot.servo_j(&command, MoveMode::ABS, 1);
        if (command_ret != 0) {
            stop_reason = "SERVO_J_FAILED";
            break;
        }
        ++commands;

        const std::uint64_t latency_ns =
            received_ns >= packet.monotonic_ns
                ? received_ns - packet.monotonic_ns
                : 0;
        latency_sum_ns += latency_ns;
        if (latency_ns > max_latency_ns) max_latency_ns = latency_ns;

        if (commands % 5 != 0) continue;

        JointValue actual{};
        const int actual_ret = robot.get_actual_joint_position(&actual);
        if (actual_ret != 0) {
            stop_reason = "TRACKING_READ_FAILED";
            break;
        }
        ++state_samples;

        double actual_position[6]{};
        double error_rad[6]{};
        for (int joint = 0; joint < 6; ++joint) {
            actual_position[joint] = actual.jVal[joint];
            error_rad[joint] =
                std::abs(target_position[joint] - actual_position[joint]);
            error_sum_rad[joint] += error_rad[joint];
            if (error_rad[joint] > max_error_rad[joint]) {
                max_error_rad[joint] = error_rad[joint];
            }
        }

        if (state_samples % 5 == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "seq=" << packet.sequence
                      << " operator_q=";
            print_joint_array(operator_position);
            std::cout << " target_q=";
            print_joint_array(target_position);
            std::cout << " actual_q=";
            print_joint_array(actual_position);
            std::cout << " error_deg=";
            print_joint_array(error_rad, kRadToDeg);
            std::cout << " latency_ms="
                      << static_cast<double>(latency_ns) / 1e6
                      << '\n';
        }
    }

    if (!running.load() && stop_reason == "PROCESS_EXIT") {
        stop_reason = "SIGNAL";
    }
    active_ended_ns = monotonic_ns();

    int abort_ret = 0;
    int servo_disable_ret = 0;
    if (servo_enabled) {
        abort_ret = robot.motion_abort();
        servo_disable_ret = robot.servo_move_enable(FALSE);
    }
    const int logout_ret = robot.login_out();
    close(socket_fd);
    unlink(socket_path.c_str());

    double average_error_deg[6]{};
    double maximum_error_deg[6]{};
    for (int joint = 0; joint < 6; ++joint) {
        average_error_deg[joint] =
            state_samples == 0
                ? 0.0
                : static_cast<double>(error_sum_rad[joint] / state_samples) *
                      kRadToDeg;
        maximum_error_deg[joint] = max_error_rad[joint] * kRadToDeg;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "six-joint follow summary commands=" << commands
              << " state_samples=" << state_samples
              << " stop_reason=" << stop_reason
              << " active_duration_sec="
              << (active_started_ns == 0
                      ? 0.0
                      : static_cast<double>(active_ended_ns -
                                            active_started_ns) /
                            1e9)
              << " avg_latency_ms="
              << (commands == 0
                      ? 0.0
                      : static_cast<double>(latency_sum_ns / commands / 1e6))
              << " max_latency_ms="
              << static_cast<double>(max_latency_ns) / 1e6
              << " avg_error_deg=[";
    for (int joint = 0; joint < 6; ++joint) {
        if (joint != 0) std::cout << ", ";
        std::cout << average_error_deg[joint];
    }
    std::cout << "] max_error_deg=[";
    for (int joint = 0; joint < 6; ++joint) {
        if (joint != 0) std::cout << ", ";
        std::cout << maximum_error_deg[joint];
    }
    std::cout << ']';
    std::cout << " abort_ret=" << abort_ret
              << " servo_disable_ret=" << servo_disable_ret
              << " logout_ret=" << logout_ret << '\n';

    return commands > 0 && abort_ret == 0 && servo_disable_ret == 0 &&
                   logout_ret == 0
               ? 0
               : 5;
}
