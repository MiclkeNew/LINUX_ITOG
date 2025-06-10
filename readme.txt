Схема подключения датчиков
[Вычислительный блок на Linux (Ubuntu 20.04)]
├── 1Gbit Ethernet → LIDAR Velodyne VLP-16
├── USB 3.0 → Камера Intel RealSense D435
├── SPI → IMU-датчик XSens MTI-1
└── CAN → Контроллеры двигателей ODrive

Необходимые библиотеки и пакеты ROS

### Основные зависимости:
```bash
# ROS Noetic (базовый)
sudo apt install ros-noetic-desktop-full

# Драйверы датчиков:
sudo apt install ros-noetic-velodyne \
                ros-noetic-realsense2-camera \
                ros-noetic-xsens-driver \
                ros-noetic-socketcan-interface

# Инструменты визуализации:
sudo apt install ros-noetic-rviz \
                ros-noetic-rqt

Скрипт выполняет:

1)Добавление репозитория ROS в sources.list

2)Импорт GPG-ключа ROS

3)Установку:

-Базовых пакетов ROS Noetic

-Специфичных драйверов для датчиков

-Инструментов разработки (catkin)

4)Создание и сборку рабочего пространства ROS

5)Инициализация environment переменных

# Пример запуска:
chmod +x install_dependencies.sh
sudo ./install_dependencies.sh

roslaunch environment_monitor sensors.launch rviz:=true