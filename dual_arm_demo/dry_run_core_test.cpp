#include "dry_run_core.hpp"
#include "latest_packet_mailbox.hpp"
#include "six_joint_mapping.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using jaka_dry_run::JointArray;
using jaka_dry_run::DryRunMetrics;
using jaka_dry_run::LoopTimingMetrics;
using jaka_dry_run::RelativeMapper;
using jaka_dry_run::ReadinessGate;
using jaka_dry_run::StatusPollScheduler;
using jaka_dry_run::Watchdog;
using jaka_dry_run::WatchdogState;

bool close_enough(double left, double right) {
    return std::abs(left - right) < 1e-12;
}

int main() {
    const double operator_mapping_zero[6]{1, 2, 3, 4, 5, 6};
    const double operator_mapping_current[6]{1.1, 1.8, 3.3, 3.6, 5.5, 5.4};
    const double tracking_mapping_zero[6]{10, 20, 30, 40, 50, 60};
    const auto mapped = jaka_six_joint::map_relative(
        operator_mapping_current,
        operator_mapping_zero,
        tracking_mapping_zero);
    const double mapped_expected[6]{10.1, 19.8, 30.3, 39.6, 50.5, 59.4};
    for (std::size_t joint = 0; joint < 6; ++joint) {
        assert(close_enough(mapped[joint], mapped_expected[joint]));
    }

    jaka_ipc::LatestPacketMailbox mailbox;
    jaka_ipc::JointSamplePacket packet{};
    packet.magic = jaka_ipc::kMagic;
    packet.version = jaka_ipc::kVersion;
    packet.size = sizeof(packet);
    packet.sequence = 10;
    mailbox.observe(packet, sizeof(packet), 1'000'000);
    packet.sequence = 12;
    mailbox.observe(packet, sizeof(packet), 9'000'000);
    const auto mailbox_snapshot = mailbox.snapshot();
    assert(mailbox_snapshot.has_packet);
    assert(mailbox_snapshot.packet.sequence == 12);
    assert(mailbox_snapshot.receive_ns == 9'000'000);
    assert(mailbox_snapshot.received == 2);
    assert(mailbox_snapshot.valid == 2);
    assert(mailbox_snapshot.sequence_gaps == 1);
    assert(close_enough(mailbox_snapshot.max_receive_interval_ms(), 8.0));

    jaka_ipc::JointSamplePacket invalid_packet = packet;
    invalid_packet.magic = 0;
    mailbox.observe(invalid_packet, sizeof(invalid_packet), 10'000'000);
    assert(mailbox.snapshot().invalid == 1);

    const JointArray signs{1, -1, 1, -1, 1, -1};
    const JointArray scales{1, 2, 1, 0.5, 1, 3};
    RelativeMapper mapper(signs, scales);

    const JointArray leader_zero{0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    const JointArray follower_zero{-0.1, -0.2, -0.3, -0.4, -0.5, -0.6};
    mapper.arm(leader_zero, follower_zero);
    assert(mapper.armed());

    const JointArray at_zero = mapper.map(leader_zero);
    for (std::size_t i = 0; i < 6; ++i) {
        assert(close_enough(at_zero[i], follower_zero[i]));
    }

    const JointArray leader_moved{0.2, 0.1, 0.5, 0.0, 0.7, 0.8};
    const JointArray target = mapper.map(leader_moved);
    const JointArray expected{-0.0, 0.0, -0.1, -0.2, -0.3, -1.2};
    for (std::size_t i = 0; i < 6; ++i) {
        assert(close_enough(target[i], expected[i]));
    }

    Watchdog watchdog;
    assert(watchdog.update(0, 0, true, 0) == WatchdogState::Waiting);
    assert(watchdog.update(1'000'000, 1'000'000, true, 0) ==
           WatchdogState::Fresh);
    assert(watchdog.update(42'000'000, 1'000'000, true, 0) ==
           WatchdogState::Hold);
    assert(watchdog.update(102'000'000, 1'000'000, true, 0) ==
           WatchdogState::Fault);
    assert(watchdog.update(103'000'000, 103'000'000, true, 0) ==
           WatchdogState::Fault);

    Watchdog sdk_error_watchdog;
    assert(sdk_error_watchdog.update(1'000'000, 1'000'000, true, -3) ==
           WatchdogState::Fault);

    DryRunMetrics metrics;
    metrics.observe(200'000, JointArray{1, 2, 3, 4, 5, 6},
                    JointArray{0.9, 2.2, 3, 4, 5, 6});
    metrics.observe(600'000, JointArray{1, 2, 3, 4, 5, 6},
                    JointArray{1, 2, 3, 4, 4.7, 6});
    assert(metrics.samples() == 2);
    assert(close_enough(metrics.average_latency_ms(), 0.4));
    assert(close_enough(metrics.max_latency_ms(), 0.6));
    assert(close_enough(metrics.max_abs_error_rad(), 0.3));

    LoopTimingMetrics timing;
    timing.observe_receive(1'000'000);
    timing.observe_receive(9'000'000);
    timing.observe_receive(39'000'000);
    timing.observe_joint_call(3'000'000);
    timing.observe_joint_call(11'000'000);
    timing.observe_status_call(2'000'000);
    timing.observe_status_call(27'000'000);
    assert(timing.receive_intervals() == 2);
    assert(close_enough(timing.max_receive_interval_ms(), 30.0));
    assert(close_enough(timing.max_joint_call_ms(), 11.0));
    assert(close_enough(timing.max_status_call_ms(), 27.0));

    ReadinessGate readiness(3);
    assert(!readiness.observe(true, true, true));
    assert(!readiness.observe(true, true, true));
    assert(readiness.observe(true, true, true));
    assert(readiness.ready());

    ReadinessGate reset_readiness(2);
    assert(!reset_readiness.observe(true, true, true));
    assert(!reset_readiness.observe(false, true, true));
    assert(!reset_readiness.observe(true, true, true));
    assert(reset_readiness.observe(true, true, true));

    StatusPollScheduler status_poll(13);
    int poll_count = 0;
    for (std::uint64_t sequence = 0; sequence < 125; ++sequence) {
        if (status_poll.should_poll(sequence)) {
            ++poll_count;
        }
    }
    assert(poll_count == 10);
    assert(status_poll.should_poll(0));
    assert(!status_poll.should_poll(1));
    assert(status_poll.should_poll(13));

    StatusPollScheduler phased_poll(25, 12);
    assert(!phased_poll.should_poll(0));
    assert(phased_poll.should_poll(12));
    assert(phased_poll.should_poll(37));

    std::cout << "DRY_RUN_CORE_TEST_OK\n";
    return 0;
}
