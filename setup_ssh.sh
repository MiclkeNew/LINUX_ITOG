#!/bin/bash

# Установка SSH сервера
sudo apt update
sudo apt install -y openssh-server

# Настройка SSH
sudo sed -i 's/#PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
sudo systemctl restart ssh

echo "SSH сервер настроен и запущен"
