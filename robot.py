#!/usr/bin/env python
import roslib
import rospy
import tf
from tf.transformations import euler_from_quaternion
import math
import copy
import numpy as np
import time

from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseWithCovariance
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry, Path, OccupancyGrid, GridCells
from std_msgs.msg import Bool
from sensor_msg.msg import Imu, LaserScan
from scipy import integrate

class Robot:
        def __init__(self):
            ''''''
            Setup Node here
            ''''''
            self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=2)
            self.laser_sub = rospy.Subscriber('/scan', LaserScan, callback=self.laser_callback)
            self.odom_sub = rospy.Subscriber('/odom', Odometry, callback=self.odom_callback)
            self.imu_sub =  rospy.Subscriber('/imu', Imu, callback=self.imu_callback)

            self.vel_msg = Twist()
            self.laser_msg = LaserScan()
            self.odom_msg = Odometry()
            self.imu_msg = Imu()

            self.odom_pose = Pose()



        def laser_callback(self, msg):

        def odom_callback(self, msg):
        def imu_callback(self, msg):
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
            while not self.odom_ready:
                pass

            # store start location
            pose_start = self.odom_pose

            keep_driving = True
            slow_down = False
            slow_way_down = False
            if self.calc_distance(pose_start) > (abs(distance) * 0.8):
                slow_down = True
            if self.calc_distance(pose_start) > (abs(distance) * 0.95):
                slow_way_down = True
            '''
            Capacity to move at different speeds for testing
            '''
            while keep_driving and not rospy.is_shutdown():
                if self.stop_nav:
                    break
                if slow_way_down:
                    # print("slowing linear a lot")
                    self.vel_msg.linear.x = math.copysign(speed, distance) * 0.2
                elif slow_down:
                    # print("slowing linear a bit")
                    self.vel_msg.linear.x = math.copysign(speed, distance) * 0.5

                self.vel_pub.publish(self.vel_msg)
                keep_driving = self.calc_distance(pose_start) < abs(distance)
                if self.calc_distance(pose_start) > (abs(distance) * 0.8):
                    slow_down = True
                if self.calc_distance(pose_start) > (abs(distance) * 0.95):
                    slow_way_down = True

            self.vel_msg.linear.x = 0
            self.vel_pub.publish(self.vel_msg)

        def rotate(self, angle):
            """
            Rotate in place
            :param angle: angle to rotate
            :return:
            """
            print("Rotating " + str(angle))
            # zero linear velocity
            self.vel_msg.linear.x = 0
            self.vel_msg.linear.y = 0
            self.vel_msg.linear.z = 0

            # set angular values
            self.vel_msg.angular.x = 0
            self.vel_msg.angular.y = 0
            self.vel_msg.angular.z = math.pi / 10

            # wait for odom
            while not self.odom_ready:
                pass

            # don't turn the long way
            # find out what range the half circle to our right is
            theta0 = self.get_yaw()
            theta1 = self.get_yaw() - math.pi
            wrap = False

            if theta1 <= (-1 * math.pi):
                # we wrap around going right, determine how to check if the yaw is in range
                theta1 = theta1 + (2 * math.pi)
                wrap = True

            if wrap:
                if theta0 >= angle or angle >= theta1:
                    self.vel_msg.angular.z *= -1
            else:
                if theta0 >= angle >= theta1:
                    self.vel_msg.angular.z *= -1

            yaw = self.get_yaw()
            turn_sign = math.copysign(1, (angle - yaw))

            stop_turn = False
            slow_down = False
            slow_way_down = False
            prev_sign = math.copysign(1, (angle - yaw))
            now_sign = math.copysign(1, (angle - yaw))
            prev_yaw = copy.deepcopy(yaw)
            now_yaw = copy.deepcopy(yaw)
            stop_turn = now_sign != prev_sign or abs(prev_yaw - now_yaw) > (2 * math.pi - 0.1) or abs(
                angle - yaw) < 0.05
            slow_down = abs(angle - yaw) < 0.3
            slow_way_down = abs(angle - yaw) < 0.1
            while not stop_turn and not rospy.is_shutdown():
                if self.stop_nav:
                    break
                # print(angle-yaw)
                if slow_way_down:
                    self.vel_msg.angular.z = math.pi / 60
                    if wrap:
                        if theta0 >= angle or angle >= theta1:
                            self.vel_msg.angular.z *= -1
                    else:
                        if theta0 >= angle >= theta1:
                            self.vel_msg.angular.z *= -1
                    # print("slowing turning a lot")
                elif slow_down:
                    self.vel_msg.angular.z = math.pi / 20
                    if wrap:
                        if theta0 >= angle or angle >= theta1:
                            self.vel_msg.angular.z *= -1
                    else:
                        if theta0 >= angle >= theta1:
                            self.vel_msg.angular.z *= -1
                    # print("slowing turning a bit")

                self.vel_pub.publish(self.vel_msg)
                yaw = self.get_yaw()
                slow_down = abs(angle - yaw) < 0.3
                slow_way_down = abs(angle - yaw) < 0.1
                prev_sign = copy.deepcopy(now_sign)
                now_sign = math.copysign(1, (angle - yaw))
                prev_yaw = copy.deepcopy(now_yaw)
                now_yaw = copy.deepcopy(yaw)
                stop_turn = now_sign != prev_sign or abs(prev_yaw - now_yaw) > (2 * math.pi - 0.1)

            self.vel_msg.angular.z = 0
            self.vel_pub.publish(self.vel_msg)
        def move_s(self, input_x, input_y, dt):
            """
            Rotate in place
            :param input_x: distance in x to drive
            :param input_y: distance in y to drive
            :param dt: time step for movement
            :return:
            """
            #ROS Imputs
            tbot_x = self.odom_pose.position.x
            tbot_y = self.odom_pose.position.y
            #Calculating Curve Parameters
            theta = 0
            angles = tf.transformations.euler_from_quaternion(
                [self.odom_pose.orientation.x,
                 self.odom_pose.orientation.y,
                 self.odom_pose.orientation.z,
                 self.odom_pose.orientation.w])
            if input_x>0
                actangle = math.atan(input_y/input_x)
            else
                if input_y>0
                    actangle = math.atan(input_y/input_x)+math.pi
                else
                    actangle = math.atan(input_y/input_x)-math.pi
            theta0 = angles[0]
            if actangle > theta0
                theta1 = theta0+2*actangle
            else
                theta1 = theta0-2*actangle

            a0 = tbot_x
            a1 = 2 * (math.tan(theta1) * input_x - input_y) / (math.tan(theta1) - math.tan(theta0))
            a2 = input_x - a1
            b0 = tbot_y
            b1 = a1 * math.tan(theta0)
            b2 = input_y - b1

            A = 4 * math.pow(a2,2) + 4 * math.pow(b2,2)
            B = 4 * a1 * a2 + 4 * b1 * b2
            C = math.pow(a1,2) + b1 ^ 2

            #foo = sqrt(A*math.pow(x,2)+B*x+C) Need to figure out quadratic
            x2 = lambda x: sqrt(A*math.pow(x,2)+B*x+C)
            dl_hat = integrate.quad(x2, 0, 1)

            #Movement
            theta = actangle-angles[0]
            self.vel_msg.linear.X = dl_hat
            self.vel_msg.angular.Z = 2*theta
            self.vel_pub.publish(self.vel_msg)
            '''
            t = time.time()
            while time.time() - t < dt
                tbot_x = self.odom_pose.position.x
                tbot_y = self.odom_pose.position.y
            '''