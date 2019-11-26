#!/usr/bin/env python
import roslib
import rospy
import math
import tf
import geometry_msgs.msg
# import transform.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion
from pathplanningmqp.msg import transform

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener()
    trans_pub = rospy.Publisher('/linear_trans', transform, queue_size = 2)
    listener.waitForTransform('/odom','/map',rospy.Time(), rospy.Duration(4.0))
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('/odom','/map',rospy.Time(0),rospy.Duration(4.0))
            (trans,rot) = listener.lookupTransform('/odom', '/map', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(trans)
        print("X Position: " + str(trans[0] * -1))
        print("Y Position: " + str(trans[1] * -1))
        trans_pub.publish(trans[1]*-1)
        rate.sleep()
