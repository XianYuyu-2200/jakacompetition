#include "JAKAZuRobot.h"
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
                  << " <robot_ip> <socket_path> [duration_sec]\n";
        return 64;
    }

    const char* robot_ip = argv[1];
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
    const int login_ret = robot.login_in(robot_ip, false);
    std::cout << "leader login_ret=" << login_ret
              << " ip=" << robot_ip << std::endl;
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

    while (running.load()) {
        const std::uint64_t cycle_ns = monotonic_ns();
        if (duration_sec > 0.0 &&
            cycle_ns - started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            break;
        }

        JointValue joints{};
        const int sdk_code = robot.get_actual_joint_position(&joints);

        jaka_ipc::JointSamplePacket packet{};
        packet.magic = jaka_ipc::kMagic;
        packet.version = jaka_ipc::kVersion;
        packet.size = sizeof(packet);
        packet.sequence = sequence++;
        packet.monotonic_ns = monotonic_ns();
        packet.sdk_code = sdk_code;

        if (sdk_code == 0) {
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
                      << "leader sequence=" << sequence
                      << " rate_hz=" << sequence / elapsed
                      << " read_ok=" << read_ok
                      << " read_errors=" << read_errors
                      << " sent=" << sent
                      << " send_errors=" << send_errors << '\n';
        }

        next_ns += period_ns;
        const std::uint64_t after_work_ns = monotonic_ns();
        if (next_ns <= after_work_ns) {
            const std::uint64_t missed =
                (after_work_ns - next_ns) / period_ns + 1;
            next_ns += missed * period_ns;
        }
        const timespec deadline = ns_to_timespec(next_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &deadline, nullptr);
    }

    const int logout_ret = robot.login_out();
    close(socket_fd);
    std::cout << "leader summary samples=" << sequence
              << " read_ok=" << read_ok
              << " read_errors=" << read_errors
              << " sent=" << sent
              << " send_errors=" << send_errors
              << " logout_ret=" << logout_ret << std::endl;

    return read_ok > 0 && sent > 0 && logout_ret == 0 ? 0 : 2;
}
