import roslib
import rospy
import math
import copy
import robot
import numpy as np

def move_s(input_x, input_y, dt)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
    imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
    vel_msg = Twist()
    laser_msg = LaserScan()
    odom_msg = Odometry()
    imu_msg = Imu()


    currentstate = recieve(odom_msg)
    return;
