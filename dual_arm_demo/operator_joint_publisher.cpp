#include "JAKAZuRobot.h"
#include "dry_run_core.hpp"
#include "joint_sample_ipc.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>

#include <sys/socket.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

namespace {

std::atomic<bool> running{true};

void handle_signal(int) {
    running.store(false);
}

std::uint64_t monotonic_ns() {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    return static_cast<std::uint64_t>(now.tv_sec) * 1'000'000'000ULL +
           static_cast<std::uint64_t>(now.tv_nsec);
}

timespec ns_to_timespec(std::uint64_t value) {
    return timespec{
        static_cast<time_t>(value / 1'000'000'000ULL),
        static_cast<long>(value % 1'000'000'000ULL),
    };
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <operator_ip> <socket_path> [duration_sec]\n";
        return 64;
    }

    const char* operator_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 4 ? std::stod(argv[3]) : 0.0;

    if (socket_path.size() >= sizeof(sockaddr_un::sun_path)) {
        std::cerr << "socket path is too long\n";
        return 65;
    }

    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    const int socket_fd = socket(AF_UNIX, SOCK_DGRAM, 0);
    if (socket_fd < 0) {
        std::cerr << "socket failed: " << std::strerror(errno) << '\n';
        return 66;
    }


    sockaddr_un destination{};
    destination.sun_family = AF_UNIX;
    std::strncpy(destination.sun_path,
                 socket_path.c_str(),
                 sizeof(destination.sun_path) - 1);

    JAKAZuRobot robot;
    const int login_ret = robot.login_in(operator_ip, false);
    std::cout << "operator login_ret=" << login_ret
              << " ip=" << operator_ip << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    constexpr std::uint64_t period_ns = 8'000'000;
    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t next_ns = started_ns;
    std::uint64_t sequence = 0;
    std::uint64_t read_ok = 0;
    std::uint64_t read_errors = 0;
    std::uint64_t sent = 0;
    std::uint64_t send_errors = 0;
    std::uint64_t last_publish_ns = 0;
    std::uint64_t max_publish_interval_ns = 0;
    std::uint64_t max_joint_call_ns = 0;
    std::uint64_t max_status_call_ns = 0;
    std::uint64_t max_drag_call_ns = 0;
    std::uint64_t missed_periods = 0;
    jaka_dry_run::StatusPollScheduler status_poll(25, 0);
    jaka_dry_run::StatusPollScheduler drag_poll(25, 12);
    RobotStatus_simple cached_status{};
    BOOL cached_dragging = FALSE;
    int cached_status_ret = -3;
    int cached_drag_ret = -3;

    while (running.load()) {
        const std::uint64_t cycle_ns = monotonic_ns();
        if (duration_sec > 0.0 &&
            cycle_ns - started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            break;
        }

        JointValue joints{};
        const std::uint64_t joint_call_started_ns = monotonic_ns();
        const int joint_ret = robot.get_actual_joint_position(&joints);
        max_joint_call_ns = std::max(
            max_joint_call_ns, monotonic_ns() - joint_call_started_ns);
        if (status_poll.should_poll(sequence)) {
            const std::uint64_t status_call_started_ns = monotonic_ns();
            cached_status_ret =
                robot.get_robot_status_simple(&cached_status);
            max_status_call_ns = std::max(
                max_status_call_ns,
                monotonic_ns() - status_call_started_ns);
        }
        if (drag_poll.should_poll(sequence)) {
            const std::uint64_t drag_call_started_ns = monotonic_ns();
            cached_drag_ret = robot.is_in_drag_mode(&cached_dragging);
            max_drag_call_ns = std::max(
                max_drag_call_ns,
                monotonic_ns() - drag_call_started_ns);
        }
        const int sdk_code = joint_ret;

        jaka_ipc::JointSamplePacket packet{};
        packet.magic = jaka_ipc::kMagic;
        packet.version = jaka_ipc::kVersion;
        packet.size = sizeof(packet);
        packet.sequence = sequence++;
        packet.monotonic_ns = monotonic_ns();
        if (last_publish_ns != 0) {
            max_publish_interval_ns = std::max(
                max_publish_interval_ns,
                packet.monotonic_ns - last_publish_ns);
        }
        last_publish_ns = packet.monotonic_ns;
        packet.sdk_code = sdk_code;
        packet.operator_powered =
            cached_status_ret == 0 && cached_status.powered_on ? 1 : 0;
        packet.operator_enabled =
            cached_status_ret == 0 && cached_status.enabled ? 1 : 0;
        packet.operator_dragging =
            cached_drag_ret == 0 && cached_dragging ? 1 : 0;
        packet.operator_valid =
            joint_ret == 0 && cached_status_ret == 0 &&
                    cached_drag_ret == 0
                ? 1
                : 0;

        if (joint_ret == 0) {
            ++read_ok;
            for (int joint = 0; joint < 6; ++joint) {
                packet.position[joint] = joints.jVal[joint];
            }
        } else {
            ++read_errors;
        }

        const ssize_t bytes = sendto(
            socket_fd,
            &packet,
            sizeof(packet),
            0,
            reinterpret_cast<const sockaddr*>(&destination),
            sizeof(destination));
        if (bytes == static_cast<ssize_t>(sizeof(packet))) {
            ++sent;
        } else {
            ++send_errors;
        }

        if (sequence % 125 == 0) {
            const double elapsed =
                static_cast<double>(monotonic_ns() - started_ns) / 1e9;
            std::cout << std::fixed << std::setprecision(2)
                      << "operator sequence=" << sequence
                      << " rate_hz=" << sequence / elapsed
                      << " read_ok=" << read_ok
                      << " read_errors=" << read_errors
                      << " sent=" << sent
                      << " send_errors=" << send_errors
                      << " powered=" << static_cast<int>(packet.operator_powered)
                      << " enabled=" << static_cast<int>(packet.operator_enabled)
                      << " dragging=" << static_cast<int>(packet.operator_dragging)
                      << '\n';
        }

        next_ns += period_ns;
        const std::uint64_t after_work_ns = monotonic_ns();
        if (next_ns <= after_work_ns) {
            const std::uint64_t missed =
                (after_work_ns - next_ns) / period_ns + 1;
            missed_periods += missed;
            next_ns += missed * period_ns;
        }
        const timespec deadline = ns_to_timespec(next_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }

    const int logout_ret = robot.login_out();
    close(socket_fd);
    std::cout << "operator summary samples=" << sequence
              << " read_ok=" << read_ok
              << " read_errors=" << read_errors
              << " sent=" << sent
              << " send_errors=" << send_errors
              << " max_publish_interval_ms="
              << std::fixed << std::setprecision(6)
              << static_cast<double>(max_publish_interval_ns) / 1e6
              << " max_operator_joint_call_ms="
              << static_cast<double>(max_joint_call_ns) / 1e6
              << " max_operator_status_call_ms="
              << static_cast<double>(max_status_call_ns) / 1e6
              << " max_operator_drag_call_ms="
              << static_cast<double>(max_drag_call_ns) / 1e6
              << " missed_periods=" << missed_periods
              << " logout_ret=" << logout_ret << std::endl;

    return read_ok > 0 && sent > 0 && logout_ret == 0 ? 0 : 2;
}
