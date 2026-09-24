#include "JAKAZuRobot.h"

#include <iomanip>
#include <iostream>

constexpr double RAD_TO_DEG = 57.29577951308232;

bool read_robot(
    const char* name,
    const char* ip,
    JAKAZuRobot& robot)
{
    RobotStatus_simple status{};
    JointValue joints{};

    const int status_ret =
        robot.get_robot_status_simple(&status);

    const int joint_ret =
        robot.get_joint_position(&joints);

    std::cout << "\n[" << name << "] " << ip << '\n';
    std::cout << "status ret=" << status_ret << '\n';

    if (status_ret == 0) {
        std::cout << "powered_on=" << status.powered_on
                  << ", enabled=" << status.enabled
                  << ", errcode=" << status.errcode
                  << '\n';
    }

    std::cout << "joint ret=" << joint_ret << '\n';

    if (joint_ret == 0) {
        std::cout << std::fixed << std::setprecision(6);

        for (int i = 0; i < 6; ++i) {
            std::cout
                << "J" << i + 1 << ": "
                << joints.jVal[i] << " rad, "
                << joints.jVal[i] * RAD_TO_DEG
                << " deg\n";
        }
    }

    return status_ret == 0 && joint_ret == 0;
}

int main()
{
    constexpr const char* MASTER_IP = "192.168.0.101";
    constexpr const char* FOLLOWER_IP = "192.168.0.102";

    JAKAZuRobot master;
    JAKAZuRobot follower;

    const int master_login =
        master.login_in(MASTER_IP, false);

    std::cout << "master login ret="
              << master_login << '\n';

    if (master_login == 0) {
        read_robot(
            "MASTER before follower login",
            MASTER_IP,
            master);
    }

    const int follower_login =
        follower.login_in(FOLLOWER_IP, false);

    std::cout << "\nfollower login ret="
              << follower_login << '\n';

    // 验证第二个连接是否覆盖第一个连接。
    if (master_login == 0) {
        read_robot(
            "MASTER after follower login",
            MASTER_IP,
            master);
    }

    if (follower_login == 0) {
        read_robot(
            "FOLLOWER",
            FOLLOWER_IP,
            follower);
    }

    if (follower_login == 0) {
        std::cout << "\nfollower logout ret="
                  << follower.login_out() << '\n';
    }

    if (master_login == 0) {
        std::cout << "master logout ret="
                  << master.login_out() << '\n';
    }

    return (master_login == 0 &&
            follower_login == 0) ? 0 : 1;
}
