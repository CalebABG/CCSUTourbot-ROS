#!/usr/bin/env bash

echo "Fixing Package Executable Files"

cd ~/ros_ws/src/razor_imu_9dof/nodes/
echo "Making Python Files in 'razor_imu_9dof' package executable"
sudo chmod +x *.py

cd ~/ros_ws/src/ccsu_tourbot/tourbot_main/src/
echo "Making Python Files in 'tourbot_main' package executable"
sudo chmod +x *.py

cd ~/ros_ws/src/ccsu_tourbot/tourbot_imu/src/
echo "Making Python Files in 'tourbot_imu' package executable"
sudo chmod +x *.py

cd ~/ros_ws/bash_scripts/
echo "Making Bash Files in '~/ros_ws/bash_scripts/' executable"
sudo chmod +x *.sh

echo "Packages Fixed! :D"
