#!/usr/bin/env python
import roslib
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math
import copy
import numpy as np
import time
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, GridCells
from std_msgs.msg import Bool
from sensor_msgs.msg import Imu, LaserScan
from scipy import integrate

def getKey():
    if os.name == 'nt':
      return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

class Robot:
        def __init__(self):
            #Setup Node here
            rospy.init_node('pathplanningmqp_robot', anonymous=True)
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
            self.laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)
            self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
            self.imu_sub =  rospy.Subscriber('/imu', Imu, callback=self.imu_callback)
            self.vel_sub = rospy.Subscriber('/cmd_vel', Twist, callback=self.cmd_vel_callback)

            self.vel_msg = Twist()
            self.laser_msg = LaserScan()
            self.odom_msg = Odometry()
            self.imu_msg = Imu()

            self.odom_pose = Pose()
            print("Robot.py Initialized")

        def cmd_vel_callback(self,msg):
            self.vel_msg = msg
        def odom_callback(self,msg):
            self.odom_msg = msg
        def laser_callback(self, msg):
            self.laser_msg = msg
        def odom_callback(self, msg):
            self.odom_msg = msg
        def imu_callback(self, msg):
            self.imu_msg = msg

        def drive_straight(self, speed, distance):
            """
            Make the robot drive straight
            :type speed: float
            :type distance: float
            :param speed: speed to drive
            :param distance: distance to drive
            :return:
            """
            # zero angular values
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = 0
            # set forward velocity
            self.vel_msg.linear.x = math.copysign(speed, distance)
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0

            # wait for odom
            # while not self.odom_ready:
            #     pass

            # store start location
            pose_start = self.odom_pose

            keep_driving = True
            slow_down = False
            slow_way_down = False
            # if self.calc_distance(pose_start) > (abs(distance) * 0.8):
            #     slow_down = True
            # if self.calc_distance(pose_start) > (abs(distance) * 0.95):
            #     slow_way_down = True
            '''
            Capacity to move at different speeds for testing
            '''
            while keep_driving and not rospy.is_shutdown():
                # if self.stop_nav:
                #     break
                if slow_way_down:
                    # print("slowing linear a lot")
                    self.vel_msg.linear.x = math.copysign(speed, distance) * 0.2
                elif slow_down:
                    # print("slowing linear a bit")
                    self.vel_msg.linear.x = math.copysign(speed, distance) * 0.5

                self.vel_pub.publish(self.vel_msg)
                # keep_driving = self.calc_distance(pose_start) < abs(distance)
                # if self.calc_distance(pose_start) > (abs(distance) * 0.8):
                #     slow_down = True
                # if self.calc_distance(pose_start) > (abs(distance) * 0.95):
                #     slow_way_down = True

            self.vel_msg.linear.x = 0
            self.vel_pub.publish(self.vel_msg)


        def move_s(self, input_x, input_y, dt):
            """
            Move a given distance in x and y for a specific time step
            :param input_x: distance in x to drive
            :param input_y: distance in y to drive
            :param dt: time step for movement
            :return:
            """
            #ROS Imputs
            tbot_x = self.odom_pose.position.x
            tbot_y = self.odom_pose.position.y
            print(tbot_x)
            #Calculating Curve Parameters
            theta = 0
            angles = tf.transformations.euler_from_quaternion(
                [self.odom_pose.orientation.x,
                 self.odom_pose.orientation.y,
                 self.odom_pose.orientation.z,
                 self.odom_pose.orientation.w])
            if input_x>0:
                actangle = math.atan(input_y/input_x)
            else:
                if input_y>0:
                    actangle = math.atan(input_y/input_x)+math.pi
                else:
                    actangle = math.atan(input_y/input_x)-math.pi
            theta0 = angles[0]
            if actangle > theta0:
                theta1 = theta0+2*actangle
            else:
                theta1 = theta0-2*actangle

            a0 = tbot_x
            a1 = 2 * (math.tan(theta1) * input_x - input_y) / (math.tan(theta1) - math.tan(theta0))
            a2 = input_x - a1
            b0 = tbot_y
            b1 = a1 * math.tan(theta0)
            b2 = input_y - b1

            A = 4 * math.pow(a2,2) + 4 * math.pow(b2,2)
            B = 4 * a1 * a2 + 4 * b1 * b2
            C = math.pow(a1,2) + math.pow(b1,2)


            x2 = lambda x: math.sqrt(A*math.pow(x,2)+B*x+C)
            [dl_hat, err] = integrate.quad(x2, 0, 1)
            print(dl_hat)

            #Movement
            theta = actangle-angles[0]
            key = '@'
            while(1):
                key = getKey()
                if key == 'e':
                    self.vel_msg.linear.x = 0
                    self.vel_msg.angular.z = 0
                    self.vel_pub.publish(self.vel_msg)
                    exit()
                    break
                self.vel_msg.linear.x = dl_hat
                self.vel_msg.angular.z = 2*theta
                self.vel_pub.publish(self.vel_msg)
                # print(self.vel_msg)
            #
            # t = time.time()
            # while time.time() - t < dt
            #     tbot_x = self.odom_pose.position.x
            #     tbot_y = self.odom_pose.position.y

if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        turtlebot = Robot()
        # turtlebot.drive_straight(0,0.1)
        turtlebot.move_s(2,2,1)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
