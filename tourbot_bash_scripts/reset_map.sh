#!/usr/bin/env bash

source ~/ros_ws/devel/setup.bash

rostopic pub -r 1 /syscommand std_msgs/String "data: 'reset'"
sleep 2
exit 1