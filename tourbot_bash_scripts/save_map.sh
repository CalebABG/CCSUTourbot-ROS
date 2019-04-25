#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

rostopic pub syscommand std_msgs/String "savegeotiff"
sleep 2
exit 1