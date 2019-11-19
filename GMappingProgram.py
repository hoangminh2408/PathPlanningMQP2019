#!/usr/bin/env python
import roslib
import rospy
import tf
from nav_msgs.msg import OccupancyGrid, MapMetaData #Map is published along /map topic
rosrun gmapping slam_gmapping scan:=scan _xmin:=-5.0 _xmax:=6.5 _ymin:=-5.0 _ymax:=6.5 #Start the gmapping program as part of init of the function

if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

class Mapper:
    def __init__(self):
        rospy.init_node('Gmapping', anonymous=True)
if __name__ == "__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
    try:
        myMapper = Mapper()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass