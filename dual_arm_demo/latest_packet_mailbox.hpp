#pragma once

#include "joint_sample_ipc.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <mutex>

namespace jaka_ipc {

struct PacketMailboxSnapshot {
    JointSamplePacket packet{};
    std::uint64_t receive_ns{0};
    std::uint64_t received{0};
    std::uint64_t valid{0};
    std::uint64_t invalid{0};
    std::uint64_t sdk_errors{0};
    std::uint64_t sequence_gaps{0};
    std::uint64_t last_receive_interval_ns{0};
    std::uint64_t max_receive_interval_ns{0};
    bool has_packet{false};

    double last_receive_interval_ms() const {
        return static_cast<double>(last_receive_interval_ns) / 1e6;
    }

    double max_receive_interval_ms() const {
        return static_cast<double>(max_receive_interval_ns) / 1e6;
    }
};

class LatestPacketMailbox {
public:
    void observe(const JointSamplePacket& packet,
                 std::size_t bytes,
                 std::uint64_t receive_ns) {
        std::lock_guard<std::mutex> lock(mutex_);
        ++snapshot_.received;
        if (bytes != sizeof(packet) || !valid_packet(packet)) {
            ++snapshot_.invalid;
            return;
        }

        ++snapshot_.valid;
        if (packet.sdk_code != 0) {
            ++snapshot_.sdk_errors;
        }
        if (snapshot_.has_packet) {
            if (packet.sequence > snapshot_.packet.sequence + 1) {
                snapshot_.sequence_gaps +=
                    packet.sequence - snapshot_.packet.sequence - 1;
            }
            if (receive_ns >= snapshot_.receive_ns) {
                snapshot_.last_receive_interval_ns =
                    receive_ns - snapshot_.receive_ns;
                snapshot_.max_receive_interval_ns = std::max(
                    snapshot_.max_receive_interval_ns,
                    snapshot_.last_receive_interval_ns);
            }
        }
        snapshot_.packet = packet;
        snapshot_.receive_ns = receive_ns;
        snapshot_.has_packet = true;
    }

    PacketMailboxSnapshot snapshot() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return snapshot_;
    }

private:
    mutable std::mutex mutex_;
    PacketMailboxSnapshot snapshot_{};
};

}  // namespace jaka_ipc
