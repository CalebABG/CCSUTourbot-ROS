#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

echo "Changing read/write (no execute) access to Motor-Controller serial port..."
sudo chmod 666 /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
echo "Success changed read/write access"

python ~/ros_ws/src/ccsu_tourbot/tourbot_main/src/gamepad_read.py
