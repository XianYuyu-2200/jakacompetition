#!/usr/bin/env bash
# JAKA Mini 2 (ROS 包内型号名: minicobo) 依赖安装
# 目标环境: Ubuntu 22.04 + ROS 2 Humble
set -euo pipefail

if [[ "${EUID}" -eq 0 ]]; then
  SUDO=""
else
  SUDO="sudo"
fi

${SUDO} apt-get update

${SUDO} apt-get install -y \
  build-essential \
  cmake \
  pkg-config \
  xdg-user-dirs \
  fonts-noto-cjk \
  python3-colcon-common-extensions \
  python3-pyqt5 \
  python3-yaml \
  python3-numpy \
  python3-opencv \
  python3-qrcode \
  python3-pyzbar \
  python3-pil \
  python3-pytest \
  libzbar0 \
  mesa-utils \
  x11-utils \
  ros-humble-cv-bridge \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-controller-manager \
  ros-humble-joint-trajectory-controller \
  ros-humble-position-controllers \
  ros-humble-moveit \
  ros-humble-moveit-configs-utils \
  ros-humble-moveit-kinematics \
  ros-humble-moveit-ros-move-group \
  ros-humble-moveit-ros-visualization \
  ros-humble-moveit-setup-assistant \
  ros-humble-moveit-ros-warehouse \
  ros-humble-moveit-visual-tools \
  ros-humble-ros-gz \
  ros-humble-ign-ros2-control \
  ignition-fortress

echo "[OK] 依赖安装完成"
