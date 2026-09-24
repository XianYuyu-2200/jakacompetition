#include "JAKAZuRobot.h"
#include "waypoint_recorder_core.hpp"

#include <array>
#include <chrono>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr int kSampleCount = 50;
constexpr int kSamplePeriodMs = 20;
constexpr double kJointSpanLimitRad = 0.003;
constexpr double kTcpSpanLimitMm = 1.0;

struct Options {
    std::string robot_ip{"192.168.0.102"};
    std::string output_path{"coffee_waypoints.yaml"};
};

void print_usage(const char* executable) {
    std::cerr << "Usage: " << executable
              << " [--robot-ip 192.168.0.102] [--output coffee_waypoints.yaml]\n";
}

bool parse_options(int argc, char* argv[], Options* options) {
    for (int i = 1; i < argc; ++i) {
        const std::string argument = argv[i];
        if ((argument == "--robot-ip" || argument == "--output") && i + 1 >= argc) {
            return false;
        }
        if (argument == "--robot-ip") {
            options->robot_ip = argv[++i];
        } else if (argument == "--output") {
            options->output_path = argv[++i];
        } else if (argument == "--help" || argument == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        } else {
            return false;
        }
    }
    return !options->robot_ip.empty() && !options->output_path.empty();
}

std::string timestamp_now() {
    const std::time_t now = std::time(nullptr);
    std::tm local{};
    localtime_r(&now, &local);
    char date[40]{};
    std::strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S%z", &local);
    std::string result = date;
    if (result.size() >= 5) result.insert(result.size() - 2, ":");
    return result;
}

bool load_repository(const std::string& path,
                     const std::string& expected_ip,
                     waypoint::Repository* repository,
                     std::string* error) {
    std::ifstream input(path);
    if (!input) return true;
    if (!repository->load(input, error)) return false;
    if (repository->robot_ip() != expected_ip) {
        *error = "existing file belongs to robot " + repository->robot_ip() +
                 ", not " + expected_ip;
        return false;
    }
    return true;
}

bool atomic_write(const std::string& path,
                  const waypoint::Repository& repository,
                  std::string* error) {
    const std::string temporary = path + ".tmp." + std::to_string(getpid());
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (!output) {
            *error = "cannot open temporary output: " + temporary;
            return false;
        }
        output << repository.to_yaml();
        output.flush();
        if (!output) {
            *error = "failed while writing temporary output: " + temporary;
            output.close();
            std::remove(temporary.c_str());
            return false;
        }
    }
    if (std::rename(temporary.c_str(), path.c_str()) != 0) {
        *error = "cannot replace output file: " + path;
        std::remove(temporary.c_str());
        return false;
    }
    return true;
}

bool read_sample(JAKAZuRobot* robot, waypoint::Sample* sample, std::string* error) {
    JointValue joints{};
    CartesianPose tcp{};
    const int joint_result = robot->get_actual_joint_position(&joints);
    const int tcp_result = robot->get_actual_tcp_position(&tcp);
    if (joint_result != 0 || tcp_result != 0) {
        std::ostringstream message;
        message << "SDK read failed: joint_ret=" << joint_result
                << " tcp_ret=" << tcp_result;
        *error = message.str();
        sample->read_ok = false;
        return false;
    }
    sample->read_ok = true;
    for (int i = 0; i < 6; ++i) sample->joints[i] = joints.jVal[i];
    sample->pose = {tcp.tran.x, tcp.tran.y, tcp.tran.z,
                    tcp.rpy.rx, tcp.rpy.ry, tcp.rpy.rz};
    return true;
}

void print_state(const waypoint::Sample& sample) {
    std::cout << std::fixed << std::setprecision(6) << "joints_rad=[";
    for (std::size_t i = 0; i < sample.joints.size(); ++i) {
        if (i) std::cout << ", ";
        std::cout << sample.joints[i];
    }
    std::cout << "]\ntcp_xyz_mm=[" << sample.pose[0] << ", " << sample.pose[1]
              << ", " << sample.pose[2] << "]\ntcp_rpy_rad=["
              << sample.pose[3] << ", " << sample.pose[4] << ", "
              << sample.pose[5] << "]\n";
}

waypoint::CaptureResult capture(JAKAZuRobot* robot) {
    std::vector<waypoint::Sample> samples;
    samples.reserve(kSampleCount);
    for (int index = 0; index < kSampleCount; ++index) {
        waypoint::Sample sample;
        std::string error;
        if (!read_sample(robot, &sample, &error)) {
            waypoint::CaptureResult failed;
            failed.error = error;
            return failed;
        }
        samples.push_back(sample);
        if (index + 1 < kSampleCount) {
            std::this_thread::sleep_for(std::chrono::milliseconds(kSamplePeriodMs));
        }
    }
    return waypoint::analyze_samples(samples, kJointSpanLimitRad, kTcpSpanLimitMm);
}

void print_help() {
    std::cout << "Commands:\n"
              << "  show            read and display the current actual state\n"
              << "  save P1         capture and save a new point (P1-P11)\n"
              << "  overwrite P1    capture and explicitly replace a point\n"
              << "  list            list saved points\n"
              << "  delete P1       delete a saved point\n"
              << "  help            show this help\n"
              << "  quit            save and exit\n";
}

void print_list(const waypoint::Repository& repository) {
    if (repository.points().empty()) {
        std::cout << "No waypoints recorded.\n";
        return;
    }
    for (const auto& entry : repository.points()) {
        std::cout << entry.first << " " << waypoint::waypoint_name(entry.first)
                  << " captured_at=" << entry.second.captured_at
                  << " confirmed=" << (entry.second.confirmed ? "true" : "false")
                  << '\n';
    }
}

bool persist_candidate(const std::string& path,
                       waypoint::Repository* repository,
                       waypoint::Repository candidate) {
    std::string error;
    if (!atomic_write(path, candidate, &error)) {
        std::cerr << "SAVE FAILED: " << error << '\n';
        return false;
    }
    *repository = std::move(candidate);
    return true;
}

}  // namespace

int main(int argc, char* argv[]) {
    Options options;
    if (!parse_options(argc, argv, &options)) {
        print_usage(argv[0]);
        return 64;
    }

    waypoint::Repository repository(options.robot_ip);
    std::string error;
    if (!load_repository(options.output_path, options.robot_ip, &repository, &error)) {
        std::cerr << "Cannot load output file: " << error << '\n';
        return 65;
    }

    JAKAZuRobot robot;
    const int login_result = robot.login_in(options.robot_ip.c_str(), false);
    if (login_result != 0) {
        std::cerr << "Robot login failed: ret=" << login_result
                  << " ip=" << options.robot_ip << '\n';
        return 1;
    }

    std::cout << "READ-ONLY waypoint recorder connected to " << options.robot_ip << '\n'
              << "Output: " << options.output_path << '\n'
              << "The program sends no robot motion commands. Type help for commands.\n";

    int exit_code = 0;
    std::string line;
    while (std::cout << "waypoint> " << std::flush, std::getline(std::cin, line)) {
        std::istringstream command_stream(line);
        std::string command;
        std::string id;
        std::string extra;
        command_stream >> command >> id >> extra;
        if (command.empty()) continue;
        if (!extra.empty()) {
            std::cerr << "Too many arguments. Type help.\n";
            continue;
        }
        if (command == "quit" || command == "exit") break;
        if (command == "help") {
            print_help();
            continue;
        }
        if (command == "list") {
            print_list(repository);
            continue;
        }
        if (command == "show") {
            waypoint::Sample sample;
            if (!read_sample(&robot, &sample, &error)) {
                std::cerr << error << '\n';
            } else {
                print_state(sample);
            }
            continue;
        }
        if (command == "delete") {
            if (!waypoint::valid_id(id)) {
                std::cerr << "Point id must be P1 through P11.\n";
                continue;
            }
            waypoint::Repository candidate = repository;
            if (!candidate.erase(id, &error)) {
                std::cerr << error << '\n';
                continue;
            }
            if (persist_candidate(options.output_path, &repository, candidate)) {
                std::cout << id << " deleted.\n";
            }
            continue;
        }
        if (command == "save" || command == "overwrite") {
            if (!waypoint::valid_id(id)) {
                std::cerr << "Point id must be P1 through P11.\n";
                continue;
            }
            if (command == "save" && repository.contains(id)) {
                std::cerr << id << " already exists; use overwrite " << id << '\n';
                continue;
            }
            if (id == "P7") {
                std::cout << "Warning: P7 is the maximum button press point. Without force "
                             "feedback, do not manually push beyond the measured button travel.\n";
            }
            std::cout << "Sampling 50 readings for 1 second. Keep the robot still...\n";
            waypoint::CaptureResult result = capture(&robot);
            if (!result.ok) {
                std::cerr << "CAPTURE REJECTED: " << result.error << '\n';
                continue;
            }
            result.waypoint.id = id;
            result.waypoint.captured_at = timestamp_now();
            result.waypoint.confirmed = false;
            waypoint::Repository candidate = repository;
            if (!candidate.save(result.waypoint, command == "overwrite", &error)) {
                std::cerr << error << '\n';
                continue;
            }
            if (persist_candidate(options.output_path, &repository, candidate)) {
                std::cout << id << " saved. max_joint_span_rad="
                          << result.maximum_joint_span_rad
                          << " max_tcp_span_mm=" << result.maximum_tcp_span_mm << '\n';
            }
            continue;
        }
        std::cerr << "Unknown command. Type help.\n";
    }

    const int logout_result = robot.login_out();
    if (logout_result != 0) {
        std::cerr << "Robot logout failed: ret=" << logout_result << '\n';
        exit_code = 2;
    }
    return exit_code;
}
