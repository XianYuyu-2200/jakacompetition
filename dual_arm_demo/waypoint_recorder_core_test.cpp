#include "waypoint_recorder_core.hpp"

#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using waypoint::CaptureResult;
using waypoint::Repository;
using waypoint::Sample;
using waypoint::Waypoint;

Sample make_sample(double offset = 0.0) {
    Sample sample{};
    sample.read_ok = true;
    for (int i = 0; i < 6; ++i) {
        sample.joints[i] = 0.1 * i + offset;
        sample.pose[i] = 100.0 + 10.0 * i + offset;
    }
    return sample;
}

std::vector<Sample> stable_samples() {
    std::vector<Sample> samples;
    for (int i = 0; i < 50; ++i) {
        samples.push_back(make_sample((i - 25) * 0.00002));
    }
    return samples;
}

int stable_test() {
    const CaptureResult result = waypoint::analyze_samples(stable_samples(), 0.003, 1.0);
    if (!result.ok || std::abs(result.waypoint.joints[2] - 0.19999) > 0.0001) {
        std::cerr << "stable samples were not accepted\n";
        return 1;
    }
    return 0;
}

int rejection_test(const std::string& mode) {
    auto samples = stable_samples();
    if (mode == "joint_jitter") {
        samples.back().joints[3] += 0.004;
    } else if (mode == "tcp_jitter") {
        samples.back().pose[1] += 1.1;
    } else if (mode == "read_failure") {
        samples[20].read_ok = false;
    }
    const CaptureResult result = waypoint::analyze_samples(samples, 0.003, 1.0);
    if (result.ok) {
        std::cerr << mode << " was accepted\n";
        return 1;
    }
    return 0;
}

int yaml_policy_test() {
    Repository repository("192.168.0.102");
    Waypoint waypoint = waypoint::analyze_samples(stable_samples(), 0.003, 1.0).waypoint;
    waypoint.id = "P1";
    waypoint.captured_at = "2026-08-28T00:00:00+08:00";
    std::string error;
    if (!repository.save(waypoint, false, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    if (repository.save(waypoint, false, &error)) {
        std::cerr << "save silently overwrote P1\n";
        return 1;
    }
    waypoint.joints[0] = 0.75;
    if (!repository.save(waypoint, true, &error)) {
        std::cerr << error << '\n';
        return 1;
    }
    Waypoint p3 = waypoint;
    p3.id = "P3";
    if (!repository.save(p3, false, &error)) {
        std::cerr << error << '\n';
        return 1;
    }

    const std::string yaml = repository.to_yaml();
    if (yaml.find("name: \"cup_below_pregrasp\"") == std::string::npos ||
        yaml.find("name: \"cup_safe_lower\"") == std::string::npos) {
        std::cerr << "P1/P3 semantic names are incorrect\n";
        return 1;
    }
    Repository loaded;
    std::istringstream input(yaml);
    if (!loaded.load(input, &error) || !loaded.contains("P1")) {
        std::cerr << "round trip failed: " << error << '\n';
        return 1;
    }
    if (std::abs(loaded.get("P1").joints[0] - 0.75) > 1e-9) {
        std::cerr << "overwritten value was not preserved\n";
        return 1;
    }
    std::cout << loaded.to_yaml();
    return 0;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc != 2) return 64;
    const std::string mode = argv[1];
    if (mode == "stable") return stable_test();
    if (mode == "joint_jitter" || mode == "tcp_jitter" || mode == "read_failure") {
        return rejection_test(mode);
    }
    if (mode == "yaml_policy") return yaml_policy_test();
    return 64;
}
