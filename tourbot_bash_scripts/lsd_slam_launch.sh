#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

roslaunch usb_cam usb_cam-test.launch &

ROS_NAMESPACE=/usb_cam rosrun image_proc image_proc &

rosrun lsd_slam_core live_slam /image:=/usb_cam/image_mono /camera_info:=/usb_cam/camera_info &

rosrun lsd_slam_viewer viewer
