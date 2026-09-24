#include "JAKAZuRobot.h"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <unistd.h>

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Usage: " << argv[0] << " <robot_ip> <label>\n";
        return 64;
    }

    const char* ip = argv[1];
    const std::string label = argv[2];
    JAKAZuRobot robot;

    std::cout << "[" << label << "] PID=" << getpid()
              << ", IP=" << ip << '\n';

    const int login_ret = robot.login_in(ip, false);
    std::cout << "[" << label << "] login_ret=" << login_ret << std::endl;
    if (login_ret != 0) {
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    bool all_reads_ok = true;

    for (int sample = 0; sample < 10; ++sample) {
        RobotStatus_simple status{};
        JointValue joints{};
        JointValue actual_joints{};
        const int status_ret = robot.get_robot_status_simple(&status);
        const int joint_ret = robot.get_joint_position(&joints);
        const int actual_ret = robot.get_actual_joint_position(&actual_joints);

        std::cout << "[" << label << "] sample=" << sample
                  << ", status_ret=" << status_ret
                  << ", joint_ret=" << joint_ret
                  << ", actual_ret=" << actual_ret;
        if (status_ret == 0) {
            std::cout << ", powered_on=" << status.powered_on
                      << ", enabled=" << status.enabled
                      << ", errcode=" << status.errcode;
        }
        std::cout << '\n';

        if (joint_ret == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "[" << label << "] joints_rad=[";
            for (int i = 0; i < 6; ++i) {
                if (i != 0) {
                    std::cout << ", ";
                }
                std::cout << joints.jVal[i];
            }
            std::cout << "]\n";
        }

        if (actual_ret == 0) {
            std::cout << std::fixed << std::setprecision(6)
                      << "[" << label << "] actual_joints_rad=[";
            for (int i = 0; i < 6; ++i) {
                if (i != 0) {
                    std::cout << ", ";
                }
                std::cout << actual_joints.jVal[i];
            }
            std::cout << "]\n";
        }

        if (status_ret != 0 || joint_ret != 0 || actual_ret != 0) {
            all_reads_ok = false;
        }
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    const int logout_ret = robot.login_out();
    std::cout << "[" << label << "] logout_ret=" << logout_ret << std::endl;
    return all_reads_ok && logout_ret == 0 ? 0 : 2;
}
