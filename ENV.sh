#!/bin/bash

# setting
user_name="orangepi"
user_password=orangepi

sudo apt update
sudo apt upgrade

sudo apt -y install pip
sudo apt -y install python2.7-dev
sudo apt -y install pybind11-dev
pip3 install matplotlib==3.5.0 -i https://pypi.tuna.tsinghua.edu.cn/simple
pip3 install pytest

sudo apt-get -y install build-essential libgtk2.0-dev libgtk-3-dev libavcodec-dev libavformat-dev libjpeg-dev libswscale-dev libtiff5-dev

# vnc setting
# https://omar2cloud.github.io/rasp/x11vnc/
cat > x11vnc.service << EOL
[Unit]
Description=x11vnc service
After=display-manager.service network.target syslog.target

[Service]
Type=simple
ExecStart=/usr/bin/x11vnc -forever -display :0 -auth guess -passwd 1
ExecStop=/usr/bin/killall x11vnc
Restart=on-failure

[Install]
WantedBy=multi-user.target
EOL

sudo apt-get -y install x11vnc
x11vnc -storepasswd
sudo mv x11vnc.service /etc/systemd/system
x11vnc -rfbport 5900 -rfbauth /home/${user_name}/.vnc/passwd -display :0 -forever -bg -repeat -nowf -capslock -shared -o /home/${user_name}/.vnc/x11vnc.log
echo ${user_password} | sudo -S systemctl daemon-reload
echo ${user_password} | sudo -S systemctl enable x11vnc.service
echo ${user_password} | sudo -S systemctl start x11vnc.service

# yaml-cpp
cd
# git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp
mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON ..
make -j8
sudo make install

# opencv 3.4.15
cd
# wget -O opencv.zip https://github.com/opencv/opencv/archive/3.4.15.zip
# unzip opencv.zip
cd opencv-3.4.15
mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D OPENCV_GENERATE_PKGCONFIG=YES ..
make -j8
sudo make install

# matplotlibcpp17
cd
cd matplotlibcpp17
mkdir build
cd build
cmake .. -DADD_DEMO=0
make -j8
sudo make install