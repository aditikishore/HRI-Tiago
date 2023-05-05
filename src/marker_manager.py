#!/usr/bin/env/python

import rospy
import time

from geometry_msgs.msg import Pose
from aruco_msgs.msg import Marker, MarkerArray



class marker_manager:
    def __init__(self):

        self.marker_list = []
        self.msg = MarkerArray()


    def get_markers(self):
        print("Please start ARUCO node within 15 sec")

        time.sleep(15)
        msg = rospy.wait_for_message("/markers", MarkerArray, timeout=None)
        self.print_msg(msg)
        self.msg = msg

        print("Message received, please shutdown node")
        time.sleep(15)

    def print_msg(self, msg):
        for marker in msg.markers:
            print("Marker ID: ", marker.id)
            print("X: ", marker.pose.pose.position.x)
            print("Y: ", marker.pose.pose.position.y)
            print("Z: ", marker.pose.pose.position.z)

        

if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('marker_man', anonymous=True)


    marker = marker_manager()
    marker.get_markers()
                


        

    
