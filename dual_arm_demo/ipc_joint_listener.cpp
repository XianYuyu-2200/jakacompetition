#include "joint_sample_ipc.hpp"

#include <atomic>
#include <cerrno>
#include <chrono>
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

enum class State {
    Waiting,
    Fresh,
    Hold,
    Fault,
};

const char* state_name(State state) {
    switch (state) {
        case State::Waiting: return "WAITING";
        case State::Fresh: return "FRESH";
        case State::Hold: return "HOLD";
        case State::Fault: return "FAULT";
    }
    return "UNKNOWN";
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2 || argc > 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <socket_path> [duration_sec]\n";
        return 64;
    }

    const std::string socket_path = argv[1];
    const double duration_sec = argc == 3 ? std::stod(argv[2]) : 0.0;
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

    unlink(socket_path.c_str());
    sockaddr_un address{};
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

    const std::uint64_t started_ns = monotonic_ns();
    std::uint64_t last_rx_ns = 0;
    std::uint64_t received = 0;
    std::uint64_t valid = 0;
    std::uint64_t invalid = 0;
    std::uint64_t sdk_errors = 0;
    std::uint64_t sequence_gaps = 0;
    std::uint64_t last_sequence = 0;
    bool have_sequence = false;
    bool fault_latched = false;
    State state = State::Waiting;

    auto set_state = [&](State next,
                         std::uint64_t age_ns,
                         const char* reason) {
        if (next != state) {
            state = next;
            std::cout << "listener state=" << state_name(state)
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
            } else {
                ++valid;
                last_rx_ns = monotonic_ns();
                if (have_sequence && packet.sequence > last_sequence + 1) {
                    sequence_gaps += packet.sequence - last_sequence - 1;
                }
                last_sequence = packet.sequence;
                have_sequence = true;
                if (packet.sdk_code != 0) {
                    ++sdk_errors;
                    fault_latched = true;
                    set_state(State::Fault, 0, "SDK_ERROR");
                } else if (!fault_latched) {
                    set_state(State::Fresh, 0, "DATA_RECEIVED");
                }

                if (valid % 125 == 0) {
                    const std::uint64_t latency_ns =
                        last_rx_ns >= packet.monotonic_ns
                            ? last_rx_ns - packet.monotonic_ns
                            : 0;
                    std::cout << std::fixed << std::setprecision(6)
                              << "listener sequence=" << packet.sequence
                              << " latency_ms="
                              << static_cast<double>(latency_ns) / 1e6
                              << " sdk_code=" << packet.sdk_code
                              << " operator_powered="
                              << static_cast<int>(packet.operator_powered)
                              << " operator_enabled="
                              << static_cast<int>(packet.operator_enabled)
                              << " operator_dragging="
                              << static_cast<int>(packet.operator_dragging)
                              << " q=[";
                    for (int joint = 0; joint < 6; ++joint) {
                        if (joint != 0) {
                            std::cout << ", ";
                        }
                        std::cout << packet.position[joint];
                    }
                    std::cout << "]\n";
                }
            }
        }

        if (last_rx_ns != 0 && !fault_latched) {
            const std::uint64_t age_ns = monotonic_ns() - last_rx_ns;
            if (age_ns >= jaka_ipc::kFaultAgeNs) {
                fault_latched = true;
                set_state(State::Fault, age_ns, "DATA_TIMEOUT");
            } else if (age_ns >= jaka_ipc::kHoldAgeNs) {
                set_state(State::Hold, age_ns, "DATA_STALE");
            }
        }
    }

    close(socket_fd);
    unlink(socket_path.c_str());
    std::cout << "listener summary received=" << received
              << " valid=" << valid
              << " invalid=" << invalid
              << " sdk_errors=" << sdk_errors
              << " sequence_gaps=" << sequence_gaps
              << " final_state=" << state_name(state) << std::endl;
    return valid > 0 && invalid == 0 ? 0 : 2;
}
