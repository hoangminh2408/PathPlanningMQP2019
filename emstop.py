#!/usr/bin/env python
import numpy as np
import rospy
import roslib
import tf
import math
from tf.transformations import euler_from_quaternion
import copy
import time
import matplotlib.pyplot as plt
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from tf.transformations import euler_from_quaternion
from pathplanningmqp.msg import transform

class lqr_controller:
    def __init__(self):
        print("Creating LQR Controller Node")
        print("............................")
        rospy.init_node('LQR_Controller')
        self.listener = tf.TransformListener()

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 2)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, callback=self.scan_callback)
        self.trans_sub = rospy.Subscriber('/linear_trans', transform, callback=self.trans_callback)
        self.odom_msg = Odometry()
        self.pose_msg = Pose()
        self.vel_msg = Twist()
        self.imu_msg = Imu()
        self.scan_msg = LaserScan()
        self.trans_msg = transform()
        self.odom_updated = False
        self.imu_updated = False
        self.scan_updated = False
        self.trans_updated = False

    def odom_callback(self, msg):
        self.odom_msg = msg
        self.odom_updated = True

    def imu_callback(self, msg):
        self.imu_msg = msg
        self.imu_updated = True

    def scan_callback(self, msg):
        self.scan_msg = msg
        self.scan_updated = True
    def trans_callback(self, msg):
        self.trans_msg = msg
        self.trans_updated = True

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        start_time = time.time()
        Robot = lqr_controller()
        for i in range(0,20):
            Robot.vel_msg.linear.x = 0
            Robot.vel_msg.angular.z = 0
            Robot.vel_pub.publish(Robot.vel_msg)
            print("STOPPING")
            rospy.sleep(1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
