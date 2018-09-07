#!/bin/bash

mkdir -p ~/Workspace/
git clone https://github.com/qutas/info)
cd ~/Workspace/info/Admin

sudo apt purge $(cat rdeps.txt)
sudo apt install $(cat deps.txt)
sudo apt install $(cat deps_ros.txt)

sudo usermod -aG $(cat ./groups_admin.txt) quas

cp ./home/quas ~/

sudo mkdir /usr/share/ros-workstation #Create shared driver folder
cp ./usr/share/ros-workstation /usr/share/ros-workstation
mkdir -p /usr/share/ros-workstation/catkin_drv/src
sudo chown -R quas:quas /usr/share/ros-workstation/	#Give write access to quas
sudo chmod a+r /usr/share/ros-workstation/ #Give read access to all
ln -s /usr/share/ros-workstation/ ~/
