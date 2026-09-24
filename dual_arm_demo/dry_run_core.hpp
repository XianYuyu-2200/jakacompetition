#pragma once

#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>

namespace jaka_dry_run {

using JointArray = std::array<double, 6>;

class RelativeMapper {
public:
    RelativeMapper(
        JointArray signs = {1, 1, 1, 1, 1, 1},
        JointArray scales = {1, 1, 1, 1, 1, 1})
        : signs_(signs), scales_(scales) {
        for (std::size_t i = 0; i < 6; ++i) {
            if (signs_[i] != 1.0 && signs_[i] != -1.0) {
                throw std::invalid_argument("joint sign must be +1 or -1");
            }
            if (scales_[i] < 0.0) {
                throw std::invalid_argument("joint scale must be non-negative");
            }
        }
    }

    void arm(const JointArray& leader_zero,
             const JointArray& follower_zero) {
        leader_zero_ = leader_zero;
        follower_zero_ = follower_zero;
        armed_ = true;
    }

    bool armed() const {
        return armed_;
    }

    const JointArray& leader_zero() const {
        return leader_zero_;
    }

    const JointArray& follower_zero() const {
        return follower_zero_;
    }

    JointArray map(const JointArray& leader) const {
        if (!armed_) {
            throw std::logic_error("relative mapper is not armed");
        }

        JointArray target{};
        for (std::size_t i = 0; i < 6; ++i) {
            target[i] = follower_zero_[i] +
                        signs_[i] * scales_[i] *
                            (leader[i] - leader_zero_[i]);
        }
        return target;
    }

private:
    JointArray signs_{};
    JointArray scales_{};
    JointArray leader_zero_{};
    JointArray follower_zero_{};
    bool armed_{false};
};

enum class WatchdogState {
    Waiting,
    Fresh,
    Hold,
    Fault,
};

class Watchdog {
public:
    WatchdogState update(
        std::uint64_t now_ns,
        std::uint64_t last_rx_ns,
        bool valid_data,
        int sdk_code) {
        if (state_ == WatchdogState::Fault) {
            return state_;
        }

        if (!valid_data || sdk_code != 0) {
            state_ = WatchdogState::Fault;
            return state_;
        }

        if (last_rx_ns == 0) {
            state_ = WatchdogState::Waiting;
            return state_;
        }

        const std::uint64_t age_ns = now_ns - last_rx_ns;
        if (age_ns >= fault_age_ns_) {
            state_ = WatchdogState::Fault;
        } else if (age_ns >= hold_age_ns_) {
            state_ = WatchdogState::Hold;
        } else {
            state_ = WatchdogState::Fresh;
        }
        return state_;
    }

    WatchdogState state() const {
        return state_;
    }

private:
    static constexpr std::uint64_t hold_age_ns_ = 40'000'000;
    static constexpr std::uint64_t fault_age_ns_ = 100'000'000;
    WatchdogState state_{WatchdogState::Waiting};
};

class ReadinessGate {
public:
    explicit ReadinessGate(std::uint32_t required_consecutive_samples = 3)
        : required_consecutive_samples_(required_consecutive_samples) {}

    bool observe(bool operator_powered,
                 bool operator_enabled,
                 bool operator_dragging) {
        const bool ready_now = operator_powered &&
                               operator_enabled &&
                               operator_dragging;
        if (!ready_now) {
            consecutive_samples_ = 0;
            ready_ = false;
            return false;
        }

        if (consecutive_samples_ < required_consecutive_samples_) {
            ++consecutive_samples_;
        }
        ready_ = consecutive_samples_ >= required_consecutive_samples_;
        return ready_;
    }

    bool ready() const {
        return ready_;
    }

private:
    std::uint32_t required_consecutive_samples_;
    std::uint32_t consecutive_samples_{0};
    bool ready_{false};
};

class StatusPollScheduler {
public:
    explicit StatusPollScheduler(std::uint64_t interval_samples,
                                 std::uint64_t phase = 0)
        : interval_samples_(interval_samples),
          phase_(interval_samples == 0 ? 0 : phase % interval_samples) {
        if (interval_samples_ == 0) {
            throw std::invalid_argument(
                "status poll interval must be non-zero");
        }
    }

    bool should_poll(std::uint64_t sequence) const {
        return sequence % interval_samples_ == phase_;
    }

private:
    std::uint64_t interval_samples_;
    std::uint64_t phase_;
};

class DryRunMetrics {
public:
    void observe(std::uint64_t latency_ns,
                 const JointArray& target,
                 const JointArray& actual) {
        ++samples_;
        latency_sum_ns_ += latency_ns;
        max_latency_ns_ = std::max(max_latency_ns_, latency_ns);
        for (std::size_t i = 0; i < 6; ++i) {
            max_abs_error_rad_ = std::max(
                max_abs_error_rad_,
                std::abs(target[i] - actual[i]));
        }
    }

    std::uint64_t samples() const {
        return samples_;
    }

    double average_latency_ms() const {
        return samples_ == 0
                   ? 0.0
                   : static_cast<double>(latency_sum_ns_) /
                         static_cast<double>(samples_) / 1e6;
    }

    double max_latency_ms() const {
        return static_cast<double>(max_latency_ns_) / 1e6;
    }

    double max_abs_error_rad() const {
        return max_abs_error_rad_;
    }

private:
    std::uint64_t samples_{0};
    std::uint64_t latency_sum_ns_{0};
    std::uint64_t max_latency_ns_{0};
    double max_abs_error_rad_{0.0};
};

class LoopTimingMetrics {
public:
    void observe_receive(std::uint64_t receive_ns) {
        if (last_receive_ns_ != 0 && receive_ns >= last_receive_ns_) {
            last_receive_interval_ns_ = receive_ns - last_receive_ns_;
            max_receive_interval_ns_ = std::max(
                max_receive_interval_ns_, last_receive_interval_ns_);
            ++receive_intervals_;
        }
        last_receive_ns_ = receive_ns;
    }

    void observe_joint_call(std::uint64_t duration_ns) {
        max_joint_call_ns_ = std::max(max_joint_call_ns_, duration_ns);
    }

    void observe_status_call(std::uint64_t duration_ns) {
        max_status_call_ns_ = std::max(max_status_call_ns_, duration_ns);
    }

    std::uint64_t receive_intervals() const {
        return receive_intervals_;
    }

    double last_receive_interval_ms() const {
        return static_cast<double>(last_receive_interval_ns_) / 1e6;
    }

    double max_receive_interval_ms() const {
        return static_cast<double>(max_receive_interval_ns_) / 1e6;
    }

    double max_joint_call_ms() const {
        return static_cast<double>(max_joint_call_ns_) / 1e6;
    }

    double max_status_call_ms() const {
        return static_cast<double>(max_status_call_ns_) / 1e6;
    }

private:
    std::uint64_t last_receive_ns_{0};
    std::uint64_t last_receive_interval_ns_{0};
    std::uint64_t max_receive_interval_ns_{0};
    std::uint64_t max_joint_call_ns_{0};
    std::uint64_t max_status_call_ns_{0};
    std::uint64_t receive_intervals_{0};
};

}  // namespace jaka_dry_run
