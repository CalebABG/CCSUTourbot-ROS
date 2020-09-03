#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

rostopic pub syscommand std_msgs/String "savegeotiff"