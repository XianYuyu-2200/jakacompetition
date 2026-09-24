#pragma once

#include <cstdint>

namespace jaka_status_ipc {

constexpr std::uint32_t kMagic = 0x53544154;
constexpr std::uint16_t kVersion = 1;

#pragma pack(push, 1)
struct StatusPacket {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t size;
    std::uint64_t sequence;
    std::uint64_t monotonic_ns;
    std::int32_t status_ret;
    std::int32_t drag_ret;
    std::uint8_t powered;
    std::uint8_t enabled;
    std::uint8_t dragging;
    std::uint8_t valid;
};
#pragma pack(pop)

static_assert(sizeof(StatusPacket) == 36);

inline bool valid_packet(const StatusPacket& packet) {
    return packet.magic == kMagic &&
           packet.version == kVersion &&
           packet.size == sizeof(StatusPacket);
}

}  // namespace jaka_status_ipc
