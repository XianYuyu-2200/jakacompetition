#pragma once

#include <array>

namespace jaka_six_joint {

inline std::array<double, 6> map_relative(
    const double operator_position[6],
    const double operator_zero[6],
    const double tracking_zero[6]) {
    std::array<double, 6> target{};
    for (int joint = 0; joint < 6; ++joint) {
        target[joint] =
            tracking_zero[joint] +
            (operator_position[joint] - operator_zero[joint]);
    }
    return target;
}

}  // namespace jaka_six_joint
