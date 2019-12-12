#!/usr/bin/env python
import roslib
import rospy
import math

roslib.load_manifest('PathPlanningMQP2019')

def path_server():

    rospy.init_node('path_service')
    s = rospy.Service('path_service', SOMETHING, SOMETHING)
    print"Ready to begin path service"
    rospy.spin()

if __name__ == "__main__":
    path_server()
