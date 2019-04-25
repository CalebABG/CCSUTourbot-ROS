#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

echo "Changing read/write (no execute) access to Motor-Controller..."
sudo chmod 666 /dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0
echo "Success changed read/write access"

echo "Changing read/write (no execute) access to LiDar..."
sudo chmod 666 /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
echo "Success changed read/write access"

# May need to change the device listed under serial/by-id if the device name changes
echo "Changing read/write (no execute) access to IMU..."
sudo chmod 666 /dev/serial/by-id/usb-SparkFun_SparkFun_SAMD21_FCF914F8504D3148372E3120FF121018-if00
echo "Success changed read/write access"

# roslaunch tourbot_main lsm.launch
roslaunch tourbot_main tourbot_scan_matcher.launch





#roslaunch tourbot_main test.launch
#roslaunch tourbot_main tourbot_run.launch


# roslaunch rplidar_ros view_slam.launch
#roslaunch hector_slam_launch tourbot_hectorsim.launch


#roslaunch hector_slam_launch hector_neato.launch
#roslaunch hector_slam_launch test1.launch
#
#~/ros_ws/run_rviz_hectorslam.sh
#gnome-terminal -e ~/ros_ws/run_rviz_hectorslam.sh
#
#gnome-terminal -e ~/ros_ws/run_rviz_hectorslam.sh
#xterm -e ~/ros_ws/run_rviz_hectorslam.sh
#
#gnome-terminal -e ~/ros_ws/run_gamepad_and_lidar_node.sh
#xterm -e ~/ros_ws/lidar_setup_perm.sh
#xterm -e ~/ros_ws/start_xv_11_lidarscan.sh
