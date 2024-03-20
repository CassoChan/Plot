#!/bin/bash

sudo apt update
sudo apt upgrade

sudo apt-get -y install python3-matplotlib
sudo apt-get -y install python3-numpy
sudo apt-get -y install python3-dev
sudo apt-get -y install python3-tk

cd matplotlib-cpp
mkdir build
cd cmake ..
make -j4
sudo make install

cd plot
mkdir build
cd build
cmake ..
make -j4