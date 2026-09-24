#pragma once

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <iomanip>
#include <istream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace waypoint {

struct Sample {
    bool read_ok{false};
    std::array<double, 6> joints{};
    std::array<double, 6> pose{};  // x, y, z in mm; rx, ry, rz in rad.
};

struct Waypoint {
    std::string id;
    std::string captured_at;
    std::array<double, 6> joints{};
    std::array<double, 6> pose{};
    bool confirmed{false};
};

struct CaptureResult {
    bool ok{false};
    std::string error;
    Waypoint waypoint;
    double maximum_joint_span_rad{0.0};
    double maximum_tcp_span_mm{0.0};
};

inline double median(std::vector<double> values) {
    std::sort(values.begin(), values.end());
    const std::size_t middle = values.size() / 2;
    if (values.size() % 2 == 0) {
        return (values[middle - 1] + values[middle]) / 2.0;
    }
    return values[middle];
}

inline CaptureResult analyze_samples(const std::vector<Sample>& samples,
                                     double joint_span_limit_rad,
                                     double tcp_span_limit_mm) {
    CaptureResult result;
    if (samples.empty()) {
        result.error = "no samples were collected";
        return result;
    }
    for (const auto& sample : samples) {
        if (!sample.read_ok) {
            result.error = "one or more SDK reads failed";
            return result;
        }
    }

    for (std::size_t dimension = 0; dimension < 6; ++dimension) {
        std::vector<double> values;
        values.reserve(samples.size());
        double minimum = samples.front().joints[dimension];
        double maximum = minimum;
        for (const auto& sample : samples) {
            const double value = sample.joints[dimension];
            values.push_back(value);
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
        }
        const double span = maximum - minimum;
        result.maximum_joint_span_rad = std::max(result.maximum_joint_span_rad, span);
        if (span > joint_span_limit_rad) {
            std::ostringstream message;
            message << "joint J" << dimension + 1 << " span " << span
                    << " rad exceeds " << joint_span_limit_rad << " rad";
            result.error = message.str();
            return result;
        }
        result.waypoint.joints[dimension] = median(std::move(values));
    }

    for (std::size_t dimension = 0; dimension < 6; ++dimension) {
        std::vector<double> values;
        values.reserve(samples.size());
        double minimum = samples.front().pose[dimension];
        double maximum = minimum;
        for (const auto& sample : samples) {
            const double value = sample.pose[dimension];
            values.push_back(value);
            minimum = std::min(minimum, value);
            maximum = std::max(maximum, value);
        }
        if (dimension < 3) {
            const double span = maximum - minimum;
            result.maximum_tcp_span_mm = std::max(result.maximum_tcp_span_mm, span);
            if (span > tcp_span_limit_mm) {
                static const char* axes[] = {"X", "Y", "Z"};
                std::ostringstream message;
                message << "TCP " << axes[dimension] << " span " << span
                        << " mm exceeds " << tcp_span_limit_mm << " mm";
                result.error = message.str();
                return result;
            }
        }
        result.waypoint.pose[dimension] = median(std::move(values));
    }
    result.ok = true;
    return result;
}

inline std::string trim(const std::string& input) {
    const auto first = std::find_if_not(input.begin(), input.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    const auto last = std::find_if_not(input.rbegin(), input.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();
    return first < last ? std::string(first, last) : std::string{};
}

inline std::string unquote(const std::string& input) {
    const std::string value = trim(input);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        return value.substr(1, value.size() - 2);
    }
    return value;
}

inline bool parse_array6(const std::string& input, std::array<double, 6>* output) {
    const auto left = input.find('[');
    const auto right = input.rfind(']');
    if (left == std::string::npos || right == std::string::npos || right <= left) return false;
    std::istringstream stream(input.substr(left + 1, right - left - 1));
    std::string token;
    std::size_t index = 0;
    try {
        while (std::getline(stream, token, ',')) {
            if (index >= output->size()) return false;
            (*output)[index++] = std::stod(trim(token));
        }
    } catch (...) {
        return false;
    }
    return index == output->size();
}

inline bool valid_id(const std::string& id) {
    if (id.size() < 2 || id.front() != 'P') return false;
    for (std::size_t i = 1; i < id.size(); ++i) {
        if (!std::isdigit(static_cast<unsigned char>(id[i]))) return false;
    }
    const int number = std::stoi(id.substr(1));
    return number >= 1 && number <= 11 && id == "P" + std::to_string(number);
}

inline std::string waypoint_name(const std::string& id) {
    static const std::map<std::string, std::string> names = {
        {"P1", "cup_below_pregrasp"},
        {"P2", "cup_grasp"},
        {"P3", "cup_safe_lower"},
        {"P4", "dispenser_preplace"},
        {"P5", "cup_under_dispenser"},
        {"P6", "button_prepress"},
        {"P7", "button_max_press"},
        {"P8", "button_retract"},
        {"P9", "filled_cup_regrasp"},
        {"P10", "filled_cup_safe_lift"},
        {"P11", "handover"},
    };
    const auto found = names.find(id);
    return found == names.end() ? std::string{} : found->second;
}

class Repository {
public:
    explicit Repository(std::string robot_ip = "192.168.0.102")
        : robot_ip_(std::move(robot_ip)) {}

    const std::string& robot_ip() const { return robot_ip_; }

    bool contains(const std::string& id) const { return points_.count(id) != 0; }

    const Waypoint& get(const std::string& id) const { return points_.at(id); }

    const std::map<std::string, Waypoint>& points() const { return points_; }

    bool save(const Waypoint& waypoint, bool overwrite, std::string* error) {
        if (!valid_id(waypoint.id)) {
            if (error) *error = "point id must be P1 through P11";
            return false;
        }
        if (contains(waypoint.id) && !overwrite) {
            if (error) *error = waypoint.id + " already exists; use overwrite explicitly";
            return false;
        }
        points_[waypoint.id] = waypoint;
        return true;
    }

    bool erase(const std::string& id, std::string* error) {
        if (points_.erase(id) == 0) {
            if (error) *error = id + " does not exist";
            return false;
        }
        return true;
    }

    std::string to_yaml() const {
        std::ostringstream output;
        output << std::setprecision(10) << std::fixed;
        output << "metadata:\n"
               << "  robot_ip: \"" << robot_ip_ << "\"\n"
               << "  coordinate_frame: \"current_controller_user_frame\"\n"
               << "  tcp_frame: \"current_controller_tool_tcp\"\n"
               << "  joint_units: \"rad\"\n"
               << "  translation_units: \"mm\"\n"
               << "  rotation_units: \"rad\"\n"
               << "  hand_state_available: false\n\n"
               << "waypoints:\n";
        for (int number = 1; number <= 11; ++number) {
            const std::string id = "P" + std::to_string(number);
            const auto found = points_.find(id);
            if (found == points_.end()) continue;
            const Waypoint& point = found->second;
            output << "  " << id << ":\n"
                   << "    name: \"" << waypoint_name(id) << "\"\n"
                   << "    captured_at: \"" << point.captured_at << "\"\n"
                   << "    joint_position_rad: [";
            write_array(output, point.joints);
            output << "]\n"
                   << "    tcp_pose:\n"
                   << "      xyz_mm: [";
            write_array3(output, point.pose, 0);
            output << "]\n"
                   << "      rpy_rad: [";
            write_array3(output, point.pose, 3);
            output << "]\n"
                   << "    hand_position: null\n"
                   << "    confirmed: " << (point.confirmed ? "true" : "false") << "\n";
        }
        return output.str();
    }

    bool load(std::istream& input, std::string* error) {
        std::map<std::string, Waypoint> loaded;
        std::string loaded_ip = robot_ip_;
        std::string current_id;
        std::string line;
        while (std::getline(input, line)) {
            const std::string value = trim(line);
            if (value.rfind("robot_ip:", 0) == 0) {
                loaded_ip = unquote(value.substr(value.find(':') + 1));
                continue;
            }
            if (line.rfind("  P", 0) == 0 && value.back() == ':') {
                current_id = value.substr(0, value.size() - 1);
                if (!valid_id(current_id)) {
                    if (error) *error = "invalid waypoint id in YAML: " + current_id;
                    return false;
                }
                Waypoint point;
                point.id = current_id;
                loaded[current_id] = point;
                continue;
            }
            if (current_id.empty()) continue;
            Waypoint& point = loaded[current_id];
            if (value.rfind("captured_at:", 0) == 0) {
                point.captured_at = unquote(value.substr(value.find(':') + 1));
            } else if (value.rfind("joint_position_rad:", 0) == 0) {
                if (!parse_array6(value, &point.joints)) {
                    if (error) *error = "invalid joint_position_rad for " + current_id;
                    return false;
                }
            } else if (value.rfind("xyz_mm:", 0) == 0) {
                if (!parse_array3(value, &point.pose, 0)) {
                    if (error) *error = "invalid xyz_mm for " + current_id;
                    return false;
                }
            } else if (value.rfind("rpy_rad:", 0) == 0) {
                if (!parse_array3(value, &point.pose, 3)) {
                    if (error) *error = "invalid rpy_rad for " + current_id;
                    return false;
                }
            } else if (value.rfind("confirmed:", 0) == 0) {
                point.confirmed = trim(value.substr(value.find(':') + 1)) == "true";
            }
        }
        for (const auto& item : loaded) {
            if (item.second.captured_at.empty()) {
                if (error) *error = "incomplete waypoint in YAML: " + item.first;
                return false;
            }
        }
        robot_ip_ = loaded_ip;
        points_ = std::move(loaded);
        return true;
    }

private:
    static void write_array(std::ostream& output, const std::array<double, 6>& values) {
        for (std::size_t i = 0; i < values.size(); ++i) {
            if (i) output << ", ";
            output << values[i];
        }
    }

    static void write_array3(std::ostream& output,
                             const std::array<double, 6>& values,
                             std::size_t offset) {
        for (std::size_t i = 0; i < 3; ++i) {
            if (i) output << ", ";
            output << values[offset + i];
        }
    }

    static bool parse_array3(const std::string& input,
                             std::array<double, 6>* output,
                             std::size_t offset) {
        const auto left = input.find('[');
        const auto right = input.rfind(']');
        if (left == std::string::npos || right == std::string::npos || right <= left) return false;
        std::istringstream stream(input.substr(left + 1, right - left - 1));
        std::string token;
        std::size_t index = 0;
        try {
            while (std::getline(stream, token, ',')) {
                if (index >= 3) return false;
                (*output)[offset + index++] = std::stod(trim(token));
            }
        } catch (...) {
            return false;
        }
        return index == 3;
    }

    std::string robot_ip_;
    std::map<std::string, Waypoint> points_;
};

}  // namespace waypoint
