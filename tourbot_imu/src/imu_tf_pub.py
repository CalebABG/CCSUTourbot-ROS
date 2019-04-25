#!/usr/bin/env python

import rospy
import tf_conversions
import tf

# from tf import transformations as T
from sensor_msgs.msg import Imu
from math import sqrt, pi
from geometry_msgs.msg import Vector3, TransformStamped


def imu_cb(msg):
    br = tf.TransformBroadcaster()

    quat = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    # eul = tf_conversions.transformations.euler_from_quaternion(quat)

    # rospy.loginfo("| Roll {}\t| Pitch {}\t| Yaw {} |".format(eul[0] * 180.0 / pi, eul[1] * 180.0 / pi, eul[2] * 180.0 / pi))

    T = TransformStamped()
    T.header.stamp = rospy.Time.now()

    T.header.frame_id = "base_link"
    T.child_frame_id = "map"

    T.transform.translation.x = 0
    T.transform.translation.y = 0
    T.transform.translation.z = 0

    T.transform.rotation.w = quat[3]
    T.transform.rotation.x = quat[0]
    T.transform.rotation.y = quat[1]
    T.transform.rotation.z = quat[2]

    br.sendTransformMessage(T)

    # br.sendTransform((0,0,0), quat, rospy.Time.now(), "imu_link", "laser")


if __name__ == '__main__':
    rospy.init_node('imu_viz')
    rospy.Subscriber('imu/data', Imu, imu_cb)
    rospy.spin()
