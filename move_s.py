import roslib
import rospy
import math
import copy
import robot
import numpy as np
import scipy.integrate as integral
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
    tbot_x = currentstate.pose.pose.position.X
    tbot_y = currentstate.pose.pose.position.Y
    quat = state.pose.pose.orientation
    angles = euler_from_quaternion(quat)
    if input_x > 0:
        actangle = np.arctan(input_y/input_x)
    else:
        if input_y > 0:
            actangle = np.arctan(input_y/input_x) + math.pi
        else:
            actangle = np.arctan(input_y/input_x) - math.pi
    theta0 = angles[0]
    if actangle > theta0:
        theta1 = theta0+2*actangle;
    else:
        theta1 = theta0-2*actangle;

    a0 = tbot_x;
    a1 = 2*(np.tan(theta1)*input_x - input_y)/(np.tan(theta1)-np.tan(theta0))
    a2 = input_x - a1
    b0 = tbot_y;
    b1 = a1*np.tan(theta0)
    b2 = input_y - b1

    A = 4*a2^2 + 4*b2^2
    B = 4*a1*a2 + 4*b1*b2
    C = a1^2+b1^2

    func = lambda x: sqrt(A*(x^2) + B * x + C)
    dl_hat = integral.quad(func,0,1)
    if np.linalg.norm(dl_hat) < np.linalg.norm([input_x, input_y]):
        print("Something is wrong")
    theta = actangle - angles[0]
    self.vel_msg.linear.X = 2*theta
    self.vel_msg.angular.Z = dl_hat
    self.vel_pub.publish(self.vel_msg)

    return;
