import roslib
import rospy
import math
import copy
import robot
import numpy as np
from tf.transformations import euler_from_quaternion
def move_s(input_x, input_y, dt)
    vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
    laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)
    odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
    imu_sub = rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
    vel_msg = Twist()
    laser_msg = LaserScan()
    odom_msg = Odometry()
    imu_msg = Imu()
    now = rospy.Time.now()
    currentstate = recieve(odom_msg)


    self.vel_msg.linear.X =
    self.vel_msg.angular.Z =
    self.vel_pub.publish(self.vel_msg)

    return;
