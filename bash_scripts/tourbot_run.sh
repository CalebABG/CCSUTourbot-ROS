#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash


echo "Changing read/write access to motor-controller serial port..."
#sudo chmod 666 /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
echo "success changed read/write access"

echo "Changing read/write access to lidar serial port..."
sudo chmod 666 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
echo "success changed read/write access"

# May need to change the device listed under serial/by-id if the device name changes
echo "Changing read/write access to imu serial port..."
sudo chmod 666 /dev/serial/by-id/usb-SparkFun_SFE_9DOF-D21_FCF914F8504D3148372E3120FF121018-if00
echo "success changed read/write access"


# roslaunch tourbot_main tourbot_scan_matcher.launch
# roslaunch tourbot_main lsm.launch


# roslaunch tourbot_main test.launch
roslaunch tourbot_main tourbot_run.launch


# roslaunch rplidar_ros view_slam.launch