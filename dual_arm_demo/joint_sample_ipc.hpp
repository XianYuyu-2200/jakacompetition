#pragma once

#include <cstddef>
#include <cstdint>

namespace jaka_ipc {

constexpr std::uint32_t kMagic = 0x31414B4A;
constexpr std::uint16_t kVersion = 2;
constexpr std::uint64_t kHoldAgeNs = 24'000'000;
constexpr std::uint64_t kFaultAgeNs = 100'000'000;

#pragma pack(push, 1)
struct JointSamplePacket {
    std::uint32_t magic;
    std::uint16_t version;
    std::uint16_t size;
    std::uint64_t sequence;
    std::uint64_t monotonic_ns;
    double position[6];
    std::int32_t sdk_code;
    std::uint8_t operator_powered;
    std::uint8_t operator_enabled;
    std::uint8_t operator_dragging;
    std::uint8_t operator_valid;
};
#pragma pack(pop)

static_assert(sizeof(JointSamplePacket) == 80);

inline bool valid_packet(const JointSamplePacket& packet) {
    return packet.magic == kMagic &&
           packet.version == kVersion &&
           packet.size == sizeof(JointSamplePacket);
}

}  // namespace jaka_ipc
