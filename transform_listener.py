#!/usr/bin/env python
import roslib
import rospy
import tf
from pathplanningmqp.msg import transform # custom transform message type is a 32bit unsigned int, used to store a single position value

if __name__ == '__main__':
    rospy.init_node('tf_listener')

    listener = tf.TransformListener() # Create transform listener to obtain map pose estimate
    trans_pub = rospy.Publisher('/linear_trans', transform, queue_size = 2) # Create publisher along linear_trans topic with custom transform message type
    listener.waitForTransform('/odom','/map',rospy.Time(), rospy.Duration(4.0))
    rate = rospy.Rate(10.0) # rate at which the tf publisher updates
    while not rospy.is_shutdown():
        try:
            listener.waitForTransform('/odom','/map',rospy.Time(0),rospy.Duration(4.0)) # Waits for a transform to be recieved, then moves onto next step. Will not progress unless tf is recieved
            (trans,rot) = listener.lookupTransform('/odom', '/map', rospy.Time(0)) # trans is the X, Y, and Z linear transform vector, while rot is the rho, theta, phi rotational transform vector
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        print(trans)
        print("X Position: " + str(trans[0] * -1)) # TF reports the position as inverted (negative when should be positive), so swap polarity
        print("Y Position: " + str(trans[1] * -1)) # TF reports the position as inverted (negative when should be positive), so swap polarity
        trans_pub.publish(trans[1]*-1) # Publish the Y transform, which is the estimated Y position of the robot
        rate.sleep()
