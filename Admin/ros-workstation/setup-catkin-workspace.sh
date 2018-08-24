#!/bin/bash

cat /usr/share/ros-workstation/bash-additions.txt >> ~/.bashrc

source /usr/share/ros-workstation/catkin_drv/devel/setup.bash

mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src

catkin_init_workspace

cd ..
catkin_make

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
