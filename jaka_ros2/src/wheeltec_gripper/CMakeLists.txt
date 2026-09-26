cmake_minimum_required(VERSION 3.8)
project(wheeltec_gripper)

find_package(ament_cmake REQUIRED)

# 纯资源包: 网格 + URDF 宏 + 实测参数, 没有可执行文件。
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY urdf/   DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY tools/  DESTINATION share/${PROJECT_NAME}/tools)
install(DIRECTORY step/   DESTINATION share/${PROJECT_NAME}/step)

ament_package()
