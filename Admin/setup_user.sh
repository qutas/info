#!/bin/bash

NEW_USER=$1

if [[ -z $NEW_USER ]]
then
	echo "Error: No username specified!"
	echo "Usage: ./setup_user.sh USERNAME"
	exit
fi

sudo -v
set -e

echo "Setting up user account: $NEW_USER"

sudo useradd -m $NEW_USER -s /bin/bash -G $(cat groups_user.txt)
echo "$NEW_USER:$NEW_USER" | sudo chpasswd
/usr/share/ros-workstation/copy-config2user.sh $NEW_USER

sudo su $NEW_USER -c "rosdep update"
sudo su $NEW_USER -c /usr/share/ros-workstation/setup-catkin-workspace.sh

sudo mkdir -p /home/$NEW_USER/Desktop/
sudo cp /usr/share/ros-workstation/QUT\ Internet\ Login.desktop /usr/share/ros-workstation/QGroundControl.desktop /home/$NEW_USER/Desktop
sudo chown $NEW_USER /home/$NEW_USER/Desktop
sudo chown $NEW_USER /home/$NEW_USER/Desktop/*
