// 常驻进程，给练习抓取反复改 Gazebo 模型位姿。
// 每帧新开一次 `ign service` 要 300ms 以上，跟手的更新会超时，网格就停在原地。
#include <iostream>
#include <string>

#include <gz/transport/Node.hh>
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/pose.pb.h>

int main(int argc, char **argv) {
  const std::string world = argc > 1 ? argv[1] : "gemini305_preview";
  const std::string service = "/world/" + world + "/set_pose";
  gz::transport::Node node;
  std::string line;
  while (std::getline(std::cin, line)) {
    char name[128];
    double x = 0, y = 0, z = 0, qx = 0, qy = 0, qz = 0, qw = 1;
    if (std::sscanf(line.c_str(), "%127s %lf %lf %lf %lf %lf %lf %lf",
                    name, &x, &y, &z, &qx, &qy, &qz, &qw) != 8) {
      std::cout << "fail" << std::endl;
      continue;
    }
    ignition::msgs::Pose req;
    req.set_name(name);
    req.mutable_position()->set_x(x);
    req.mutable_position()->set_y(y);
    req.mutable_position()->set_z(z);
    req.mutable_orientation()->set_x(qx);
    req.mutable_orientation()->set_y(qy);
    req.mutable_orientation()->set_z(qz);
    req.mutable_orientation()->set_w(qw);
    ignition::msgs::Boolean rep;
    bool result = false;
    const bool sent = node.Request(service, req, 300u, rep, result);
    std::cout << ((sent && result && rep.data()) ? "ok" : "fail") << std::endl;
  }
  return 0;
}
