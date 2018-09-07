#!/bin/bash

NEW_USER=$1

if [[ -z $NEW_USER ]]
then
	echo "Error: No username specified!"
	exit
fi

sudo -v
set -e

echo "Setting up user account: $NEW_USER"

#sudo useradd -m $NEW_USER -p $NEW_USER
sudo usermod -aG $(cat groups_user.txt) $NEW_USER
#/usr/share/ros-workstation/copy-config2user.sh $NEW_USER
#sudo chsh -s /bin/bash $NEW_USER
#sudo su $NEW_USER -c /usr/share/ros-workstation/setup-catkin-workspace.sh
sudo su $NEW_USER -c /usr/share/ros-workstation/setup-catkin-workspace.sh
cp /usr/share/ros-workstation/QUT\ Internet\ Login.desktop /usr/share/ros-workstation/QGroundControl.desktop /home/$NEW_USER/Desktop
sudo chown $NEW_USER /home/$NEW_USER/Desktop/*