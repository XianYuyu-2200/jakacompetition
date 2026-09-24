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

#include <poll.h>
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

const char* state_name(jaka_dry_run::WatchdogState state) {
    switch (state) {
        case jaka_dry_run::WatchdogState::Waiting: return "WAITING";
        case jaka_dry_run::WatchdogState::Fresh: return "FRESH";
        case jaka_dry_run::WatchdogState::Hold: return "HOLD";
        case jaka_dry_run::WatchdogState::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <follower_ip> <socket_path> [duration_sec]\n";
        return 64;
    }

    const char* follower_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc == 4 ? std::stod(argv[3]) : 0.0;

    sockaddr_un address{};
    if (socket_path.size() >= sizeof(address.sun_path)) {
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

    unlink(socket_path.c_str());
    address.sun_family = AF_UNIX;
    std::strncpy(address.sun_path,
                 socket_path.c_str(),
                 sizeof(address.sun_path) - 1);
    if (bind(socket_fd,
             reinterpret_cast<const sockaddr*>(&address),
             sizeof(address)) != 0) {
        std::cerr << "bind failed: " << std::strerror(errno) << '\n';
        close(socket_fd);
        return 67;
    }

    JAKAZuRobot follower;
    const int login_ret = follower.login_in(follower_ip, false);
    std::cout << "follower dry-run login_ret=" << login_ret
              << " ip=" << follower_ip << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        unlink(socket_path.c_str());
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    jaka_dry_run::RelativeMapper mapper;
    jaka_dry_run::Watchdog watchdog;
    jaka_dry_run::DryRunMetrics metrics;
    jaka_dry_run::WatchdogState state = watchdog.state();
    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t last_rx_ns = 0;
    std::uint64_t received = 0;
    std::uint64_t valid = 0;
    std::uint64_t invalid = 0;
    std::uint64_t sdk_errors = 0;
    std::uint64_t follower_read_errors = 0;
    std::uint64_t sequence_gaps = 0;
    std::uint64_t last_sequence = 0;
    bool have_sequence = false;
    bool have_follower_pose = false;
    jaka_dry_run::JointArray follower_actual{};

    auto set_state = [&](jaka_dry_run::WatchdogState next,
                         std::uint64_t age_ns,
                         const char* reason) {
        if (next != state) {
            state = next;
            std::cout << "dry-run state=" << state_name(state)
                      << " age_ms=" << std::fixed << std::setprecision(3)
                      << static_cast<double>(age_ns) / 1e6
                      << " reason=" << reason << std::endl;
        }
    };

    while (running.load()) {
        const std::uint64_t now_ns = monotonic_ns();
        if (duration_sec > 0.0 &&
            now_ns - started_ns >=
                static_cast<std::uint64_t>(duration_sec * 1e9)) {
            break;
        }

        pollfd descriptor{socket_fd, POLLIN, 0};
        const int poll_ret = poll(&descriptor, 1, 5);
        if (poll_ret > 0 && (descriptor.revents & POLLIN)) {
            jaka_ipc::JointSamplePacket packet{};
            const ssize_t bytes = recv(socket_fd, &packet, sizeof(packet), 0);
            ++received;

            if (bytes != static_cast<ssize_t>(sizeof(packet)) ||
                !jaka_ipc::valid_packet(packet)) {
                ++invalid;
                set_state(jaka_dry_run::WatchdogState::Fault,
                          0,
                          "INVALID_PACKET");
            } else if (packet.sdk_code != 0) {
                ++valid;
                ++sdk_errors;
                set_state(jaka_dry_run::WatchdogState::Fault,
                          0,
                          "LEADER_SDK_ERROR");
            } else {
                ++valid;
                last_rx_ns = monotonic_ns();

                if (have_sequence && packet.sequence > last_sequence + 1) {
                    sequence_gaps += packet.sequence - last_sequence - 1;
                }
                last_sequence = packet.sequence;
                have_sequence = true;

                JointValue joints{};
                const int follower_ret =
                    follower.get_actual_joint_position(&joints);
                if (follower_ret != 0) {
                    ++follower_read_errors;
                    set_state(jaka_dry_run::WatchdogState::Fault,
                              0,
                              "FOLLOWER_SDK_ERROR");
                } else {
                    for (int joint = 0; joint < 6; ++joint) {
                        follower_actual[joint] = joints.jVal[joint];
                    }
                    have_follower_pose = true;

                    if (!mapper.armed()) {
                        jaka_dry_run::JointArray leader_zero{};
                        for (int joint = 0; joint < 6; ++joint) {
                            leader_zero[joint] = packet.position[joint];
                        }
                        mapper.arm(leader_zero, follower_actual);
                        std::cout << "dry-run mapper=ARMED"
                                  << " leader_sequence="
                                  << packet.sequence << std::endl;
                    }

                    if (state != jaka_dry_run::WatchdogState::Fault) {
                        set_state(jaka_dry_run::WatchdogState::Fresh,
                                  0,
                                  "DATA_RECEIVED");
                    }

                    if (mapper.armed()) {
                        const auto target = mapper.map(
                            jaka_dry_run::JointArray{
                                packet.position[0], packet.position[1],
                                packet.position[2], packet.position[3],
                                packet.position[4], packet.position[5]});
                        const std::uint64_t latency_ns =
                            last_rx_ns >= packet.monotonic_ns
                                ? last_rx_ns - packet.monotonic_ns
                                : 0;
                        metrics.observe(latency_ns, target, follower_actual);
                        if (valid % 125 == 0) {
                            std::cout << std::fixed << std::setprecision(6)
                                      << "dry-run sequence=" << packet.sequence
                                      << " latency_ms="
                                      << static_cast<double>(latency_ns) / 1e6
                                      << " leader_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << packet.position[joint];
                            }
                            std::cout << "] follower_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << follower_actual[joint];
                            }
                            std::cout << "] target_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << target[joint];
                            }
                            std::cout << "] error_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << target[joint] - follower_actual[joint];
                            }
                            std::cout << "]\n";
                        }
                    }
                }
            }
        }

        if (last_rx_ns != 0 && state != jaka_dry_run::WatchdogState::Fault) {
            const std::uint64_t age_ns = monotonic_ns() - last_rx_ns;
            if (age_ns >= 100'000'000) {
                set_state(jaka_dry_run::WatchdogState::Fault,
                          age_ns,
                          "DATA_TIMEOUT");
            } else if (age_ns >= 24'000'000) {
                set_state(jaka_dry_run::WatchdogState::Hold,
                          age_ns,
                          "DATA_STALE");
            }
        }
    }

    const int logout_ret = follower.login_out();
    close(socket_fd);
    unlink(socket_path.c_str());
    std::cout << "dry-run summary received=" << received
              << " valid=" << valid
              << " invalid=" << invalid
              << " sdk_errors=" << sdk_errors
              << " follower_read_errors=" << follower_read_errors
              << " sequence_gaps=" << sequence_gaps
              << " metrics_samples=" << metrics.samples()
              << " avg_latency_ms=" << std::fixed << std::setprecision(6)
              << metrics.average_latency_ms()
              << " max_latency_ms=" << metrics.max_latency_ms()
              << " max_abs_error_rad=" << metrics.max_abs_error_rad()
              << " mapper_armed=" << (mapper.armed() ? 1 : 0)
              << " final_state=" << state_name(state)
              << " follower_pose_valid=" << (have_follower_pose ? 1 : 0)
              << " logout_ret=" << logout_ret << std::endl;

    return valid > 0 && invalid == 0 && sdk_errors == 0 &&
               follower_read_errors == 0 && mapper.armed() &&
               logout_ret == 0
           ? 0
           : 2;
}
