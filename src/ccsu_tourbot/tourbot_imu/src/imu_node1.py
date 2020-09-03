#!/usr/bin/env python

import math
import serial
import struct
import sys
from collections import namedtuple

import rospy
from sensor_msgs.msg import Imu

# a (accel) for linear accel, g (gyro) for angular velocity
razorimu = namedtuple('razorimu', 'ax ay az mx my mz gx gy gz')

degrees2rad = math.pi / 180.0
imu_yaw_calibration = 0.0

angular_velocity_covariance = 0.02  # indexes: 0,4,8
linear_acceleration_covariance = 0.04
magnetic_field_covariance = 0.0
orientation_covariance = 0.0025

# whether or not to publish imu/data topic
publish_imu_data_topic = True

# initialize ROS node and publishers
rospy.init_node("sparkfun_imu_node")
# rate = rospy.Rate(10)

# most recent measurement, i.e. queue_size=1
imu_data_raw_pub = rospy.Publisher('imu/data_raw', Imu, queue_size=1)

# port and frame_id
imu_frame_id = 'imu_link'
default_port = '/dev/serial/by-id/usb-SparkFun_SparkFun_SAMD21_FCF914F8504D3148372E3120FF121018-if00'

# setup your port and baud rate
rospy.loginfo("Opening Port: %s" % default_port)
try:
    ser = serial.Serial(port=default_port, baudrate=57600, timeout=1)
except serial.serialutil.SerialException:
    rospy.logerr("IMU not found at port " + default_port + ". Did you specify the correct port in the launch file?")
    sys.exit(-1)

# needed vars
calibrate_sensor_offset = False
print_calibration = False

nlchar = chr(13)
imu_init_time = 3  # in seconds

# sensor reports accel as 256.0 = 1G (9.8m/s^2). Convert to m/s^2.
# accel_factor = 9.80665 / 256.0
accel_factor = 9.81 / 256.0

rospy.loginfo("Giving the razor IMU board %s seconds to boot..." % imu_init_time)
rospy.sleep(imu_init_time)  # Sleep for ... seconds to wait for the board to boot

### configure board ###
# stop datastream
ser.write('#o0' + nlchar)

# disable output of sensor errors
ser.write('#oe0' + nlchar)

discard = ser.readlines()

# set imu output
ser.write('#oscb' + nlchar)

# print calibration values for verification by user
ser.flushInput()
ser.write('#p' + nlchar)

calib_data = ser.readlines()
calib_data_print = "Printing set calibration values:\r\n"
for line in calib_data:
    calib_data_print += line
if print_calibration:
    rospy.loginfo(calib_data_print)

# start datastream
ser.write('#o1' + nlchar)

rospy.loginfo("Publishing IMU data...")


while not rospy.is_shutdown():
    # read 36 bytes from IMU because 9 axis binary output sends packets of 9 floats (4 bytes each)
    # '<' because the IMU sends packets in little-endian format
    data = razorimu._make(struct.unpack('<9f', ser.read(36)))

    # IMU imu/data_raw msg
    imu_raw_msg = Imu()

    # set headers
    imu_raw_msg.header.stamp = rospy.Time.now()
    imu_raw_msg.header.frame_id = imu_frame_id

    ### Accel ###
    # This means y and z are correct for ROS, but x needs reversing
    imu_raw_msg.linear_acceleration.x = -data.ax * accel_factor
    # imu_raw_msg.linear_acceleration.x = data.ax * accel_factor
    imu_raw_msg.linear_acceleration.y = data.ay * accel_factor
    imu_raw_msg.linear_acceleration.z = data.az * accel_factor

    ### Gyro ###
    imu_raw_msg.angular_velocity.x = data.gx
    # in AHRS firmware y axis points right, in ROS y axis points left (see REP 103)
    imu_raw_msg.angular_velocity.y = -data.gy
    # imu_raw_msg.angular_velocity.y = data.gy
    # in AHRS firmware z axis points down, in ROS z axis points up (see REP 103)
    imu_raw_msg.angular_velocity.z = -data.gz
    # imu_raw_msg.angular_velocity.z = data.gz


    ### publish raw data ###
    imu_data_raw_pub.publish(imu_raw_msg)


ser.close()
