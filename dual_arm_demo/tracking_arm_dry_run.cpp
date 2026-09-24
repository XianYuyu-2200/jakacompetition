#include "JAKAZuRobot.h"
#include "dry_run_core.hpp"
#include "joint_sample_ipc.hpp"
#include "latest_packet_mailbox.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstring>
#include <fstream>
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
    if (argc < 3 || argc > 5) {
        std::cerr << "Usage: " << argv[0]
                  << " <tracking_ip> <socket_path> [duration_sec] [csv_path]\n";
        return 64;
    }

    const char* tracking_ip = argv[1];
    const std::string socket_path = argv[2];
    const double duration_sec = argc >= 4 ? std::stod(argv[3]) : 0.0;
    const std::string csv_path = argc == 5 ? argv[4] : "";
    std::ofstream csv;
    if (!csv_path.empty()) {
        csv.open(csv_path, std::ios::out | std::ios::trunc);
        if (!csv) {
            std::cerr << "failed to open csv: " << csv_path << '\n';
            return 68;
        }
        csv << "sequence,rx_monotonic_ns,receive_interval_ms,latency_ms,"
               "operator_powered,operator_enabled,operator_dragging,"
               "tracking_powered,mapper_armed,"
               "operator_j1,operator_j2,operator_j3,operator_j4,operator_j5,operator_j6,"
               "tracking_j1,tracking_j2,tracking_j3,tracking_j4,tracking_j5,tracking_j6,"
               "target_j1,target_j2,target_j3,target_j4,target_j5,target_j6\n";
    }

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

    JAKAZuRobot tracking_arm;
    const int login_ret = tracking_arm.login_in(tracking_ip, false);
    std::cout << "tracking dry-run login_ret=" << login_ret
              << " ip=" << tracking_ip << std::endl;
    if (login_ret != 0) {
        close(socket_fd);
        unlink(socket_path.c_str());
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));

    jaka_dry_run::RelativeMapper mapper;
    jaka_dry_run::ReadinessGate operator_readiness(25);
    jaka_dry_run::DryRunMetrics metrics;
    jaka_dry_run::LoopTimingMetrics timing;
    jaka_dry_run::StatusPollScheduler tracking_status_poll(25, 0);
    jaka_dry_run::WatchdogState state =
        jaka_dry_run::WatchdogState::Waiting;
    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t tracking_read_errors = 0;
    std::uint64_t processed_samples = 0;
    std::uint64_t processing_skips = 0;
    std::uint64_t last_processed_sequence = 0;
    std::uint64_t observed_invalid = 0;
    std::uint64_t observed_sdk_errors = 0;
    bool have_processed_sequence = false;
    bool have_tracking_pose = false;
    jaka_dry_run::JointArray tracking_actual{};
    RobotStatus_simple tracking_status{};
    int tracking_status_ret = -1;
    jaka_ipc::LatestPacketMailbox mailbox;

    std::thread receiver_thread([&]() {
        while (running.load()) {
            pollfd descriptor{socket_fd, POLLIN, 0};
            const int poll_ret = poll(&descriptor, 1, 5);
            if (poll_ret > 0 && (descriptor.revents & POLLIN)) {
                jaka_ipc::JointSamplePacket packet{};
                const ssize_t bytes =
                    recv(socket_fd, &packet, sizeof(packet), 0);
                mailbox.observe(packet,
                                bytes > 0 ? static_cast<std::size_t>(bytes) : 0,
                                monotonic_ns());
            }
        }
    });

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

        const auto packet_snapshot = mailbox.snapshot();
        if (packet_snapshot.invalid > observed_invalid) {
            observed_invalid = packet_snapshot.invalid;
            set_state(jaka_dry_run::WatchdogState::Fault,
                      0,
                      "INVALID_PACKET");
        }
        if (packet_snapshot.sdk_errors > observed_sdk_errors) {
            observed_sdk_errors = packet_snapshot.sdk_errors;
            set_state(jaka_dry_run::WatchdogState::Fault,
                      0,
                      "LEADER_SDK_ERROR");
        }

        if (packet_snapshot.has_packet &&
            (!have_processed_sequence ||
             packet_snapshot.packet.sequence != last_processed_sequence)) {
            const auto packet = packet_snapshot.packet;
            if (have_processed_sequence &&
                packet.sequence > last_processed_sequence + 1) {
                processing_skips +=
                    packet.sequence - last_processed_sequence - 1;
            }
            last_processed_sequence = packet.sequence;
            have_processed_sequence = true;
            ++processed_samples;

            if (packet.sdk_code != 0) {
                set_state(jaka_dry_run::WatchdogState::Fault,
                          0,
                          "LEADER_SDK_ERROR");
            } else {
                JointValue joints{};
                const std::uint64_t joint_call_started_ns = monotonic_ns();
                const int tracking_joint_ret =
                    tracking_arm.get_actual_joint_position(&joints);
                timing.observe_joint_call(
                    monotonic_ns() - joint_call_started_ns);
                if (tracking_status_poll.should_poll(packet.sequence) ||
                    tracking_status_ret != 0) {
                    const std::uint64_t status_call_started_ns = monotonic_ns();
                    tracking_status_ret =
                        tracking_arm.get_robot_status_simple(&tracking_status);
                    timing.observe_status_call(
                        monotonic_ns() - status_call_started_ns);
                }
                const bool operator_ready = operator_readiness.observe(
                    packet.operator_powered != 0,
                    packet.operator_enabled != 0,
                    packet.operator_dragging != 0);
                const bool tracking_ready =
                    tracking_status_ret == 0 &&
                    tracking_status.powered_on != 0 &&
                    tracking_joint_ret == 0;

                if (tracking_joint_ret != 0 || tracking_status_ret != 0) {
                    ++tracking_read_errors;
                    set_state(jaka_dry_run::WatchdogState::Fault,
                              0,
                              "TRACKING_SDK_ERROR");
                } else {
                    for (int joint = 0; joint < 6; ++joint) {
                        tracking_actual[joint] = joints.jVal[joint];
                    }
                    have_tracking_pose = true;

                    if (!mapper.armed() && operator_ready && tracking_ready) {
                        jaka_dry_run::JointArray leader_zero{};
                        for (int joint = 0; joint < 6; ++joint) {
                            leader_zero[joint] = packet.position[joint];
                        }
                        mapper.arm(leader_zero, tracking_actual);
                        std::cout << "dry-run mapper=ARMED"
                                  << " leader_sequence="
                                  << packet.sequence << std::endl;
                    } else if (!mapper.armed() && processed_samples % 125 == 0) {
                        std::cout << "dry-run mapper=WAITING"
                                  << " operator_ready=" << (operator_ready ? 1 : 0)
                                  << " tracking_ready=" << (tracking_ready ? 1 : 0)
                                  << " operator_powered="
                                  << static_cast<int>(packet.operator_powered)
                                  << " operator_enabled="
                                  << static_cast<int>(packet.operator_enabled)
                                  << " operator_dragging="
                                  << static_cast<int>(packet.operator_dragging)
                                  << " tracking_powered="
                                  << tracking_status.powered_on << std::endl;
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
                            packet_snapshot.receive_ns >= packet.monotonic_ns
                                ? packet_snapshot.receive_ns - packet.monotonic_ns
                                : 0;
                        metrics.observe(latency_ns, target, tracking_actual);
                        if (csv) {
                            csv << packet.sequence << ','
                                << packet_snapshot.receive_ns << ','
                                << std::fixed << std::setprecision(6)
                                << packet_snapshot.last_receive_interval_ms() << ','
                                << static_cast<double>(latency_ns) / 1e6 << ','
                                << static_cast<int>(packet.operator_powered) << ','
                                << static_cast<int>(packet.operator_enabled) << ','
                                << static_cast<int>(packet.operator_dragging) << ','
                                << tracking_status.powered_on << ','
                                << (mapper.armed() ? 1 : 0);
                            for (int joint = 0; joint < 6; ++joint) {
                                csv << ',' << packet.position[joint];
                            }
                            for (int joint = 0; joint < 6; ++joint) {
                                csv << ',' << tracking_actual[joint];
                            }
                            for (int joint = 0; joint < 6; ++joint) {
                                csv << ',' << target[joint];
                            }
                            csv << '\n';
                        }
                        if (processed_samples % 125 == 0) {
                            std::cout << std::fixed << std::setprecision(6)
                                      << "dry-run sequence=" << packet.sequence
                                      << " latency_ms="
                                      << static_cast<double>(latency_ns) / 1e6
                                      << " leader_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << packet.position[joint];
                            }
                            std::cout << "] tracking_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << tracking_actual[joint];
                            }
                            std::cout << "] target_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << target[joint];
                            }
                            std::cout << "] error_q=[";
                            for (int joint = 0; joint < 6; ++joint) {
                                if (joint != 0) std::cout << ", ";
                                std::cout << target[joint] - tracking_actual[joint];
                            }
                            std::cout << "]\n";
                        }
                    }
                }
            }
        }

        if (packet_snapshot.has_packet &&
            state != jaka_dry_run::WatchdogState::Fault) {
            const std::uint64_t age_ns =
                monotonic_ns() - packet_snapshot.receive_ns;
            if (age_ns >= 100'000'000) {
                set_state(jaka_dry_run::WatchdogState::Fault,
                          age_ns,
                          "DATA_TIMEOUT");
            } else if (age_ns >= 40'000'000) {
                set_state(jaka_dry_run::WatchdogState::Hold,
                          age_ns,
                          "DATA_STALE");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    running.store(false);
    receiver_thread.join();
    const auto final_packet_snapshot = mailbox.snapshot();
    const int logout_ret = tracking_arm.login_out();
    close(socket_fd);
    unlink(socket_path.c_str());
    std::cout << "dry-run summary received="
              << final_packet_snapshot.received
              << " valid=" << final_packet_snapshot.valid
              << " invalid=" << final_packet_snapshot.invalid
              << " sdk_errors=" << final_packet_snapshot.sdk_errors
              << " tracking_read_errors=" << tracking_read_errors
              << " sequence_gaps=" << final_packet_snapshot.sequence_gaps
              << " processed_samples=" << processed_samples
              << " processing_skips=" << processing_skips
              << " metrics_samples=" << metrics.samples()
              << " avg_latency_ms=" << std::fixed << std::setprecision(6)
              << metrics.average_latency_ms()
              << " max_latency_ms=" << metrics.max_latency_ms()
              << " max_abs_error_rad=" << metrics.max_abs_error_rad()
              << " max_receive_interval_ms="
              << final_packet_snapshot.max_receive_interval_ms()
              << " max_tracking_joint_call_ms="
              << timing.max_joint_call_ms()
              << " max_tracking_status_call_ms="
              << timing.max_status_call_ms()
              << " mapper_armed=" << (mapper.armed() ? 1 : 0)
              << " final_state=" << state_name(state)
              << " tracking_pose_valid=" << (have_tracking_pose ? 1 : 0)
              << " logout_ret=" << logout_ret << std::endl;

    return final_packet_snapshot.valid > 0 &&
               final_packet_snapshot.invalid == 0 &&
               final_packet_snapshot.sdk_errors == 0 &&
               tracking_read_errors == 0 &&
               logout_ret == 0
           ? 0
           : 2;
}
