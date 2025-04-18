FROM ros:noetic

# Установка зависимостей
RUN apt-get update && apt-get install -y \
    openssh-server \
    ros-noetic-velodyne \
    ros-noetic-realsense2-camera \
    ros-noetic-xsens-driver \
    ros-noetic-socketcan-interface \
    && rm -rf /var/lib/apt/lists/*

# Настройка SSH
RUN mkdir /var/run/sshd
RUN echo 'root:password' | chpasswd
RUN sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Создание рабочего пространства ROS
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; catkin_make'

# Копирование пакета
COPY ./catkin_ws/src/environment_monitor /catkin_ws/src/environment_monitor
RUN /bin/bash -c '. /opt/ros/noetic/setup.bash; cd /catkin_ws; catkin_make'

# Запуск SSH
EXPOSE 22
CMD ["/usr/sbin/sshd", "-D"]
