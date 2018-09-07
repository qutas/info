#!/bin/bash

cd ~/ros-workstation/catkin_drv/src
catkin_init_workspace
cd ..
catkin_make

cd ./src
cat ~/Workspace/info/Admin/catkin_drv_pkgs.txt | while read in; do git clone "https://github.com/qutas/$in"; done
cd ..
catkin_make