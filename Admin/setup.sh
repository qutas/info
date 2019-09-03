#!/bin/bash

sudo -v

sudo apt purge $(cat rdeps.txt)
sudo apt install $(cat deps.txt)
sudo apt install $(cat deps_ros.txt)

sudo cp ./etc/chrony/chrony.conf /etc/chrony/chrony.conf
sudo systemctl restart chrony.service

sudo usermod -aG $(cat ./groups_admin.txt) quas

xfce4-panel --quit
pkill xfconfd
cp -R ./home/quas/* ~/
xfce4-panel &

sudo mkdir /usr/share/ros-workstation #Create shared driver folder
sudo chown -R quas:quas /usr/share/ros-workstation/	#Give write access to quas
sudo chmod a+r /usr/share/ros-workstation/ #Give read access to all

cp -r ./usr/share/ros-workstation/* /usr/share/ros-workstation
mkdir -p /usr/share/ros-workstation/catkin_drv/src
ln -s /usr/share/ros-workstation/ ~/
