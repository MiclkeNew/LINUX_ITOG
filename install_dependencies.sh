#!/bin/bash

# Установка ROS Noetic
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
sudo apt install -y ros-noetic-desktop-full

# Установка зависимостей для работы с датчиками
sudo apt install -y ros-noetic-velodyne \
                    ros-noetic-realsense2-camera \
                    ros-noetic-xsens-driver \
                    ros-noetic-socketcan-interface

# Инициализация рабочего пространства ROS
source /opt/ros/noetic/setup.bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash

echo "Установка завершена!"
