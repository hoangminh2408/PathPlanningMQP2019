#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg


if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            (trans,rot) = listener.lookupTransform('/odom', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #trans = list of x,y,z translations
        #rot = list of x,y,z,w rotations
        print(trans)
        print("X Position" + trans[0])
        print("Y Position" + trans[1])
        rate.sleep()