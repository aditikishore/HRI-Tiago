#!/usr/bin/env/python

import rospy
import time

from geometry_msgs.msg import Pose, PoseStamped
from aruco_msgs.msg import Marker, MarkerArray

import tf

    

class marker_manager:
    def __init__(self):

        self.marker_list = []
        self.msg = MarkerArray()
        self.tf_listener = tf.TransformListener()


    def get_markers(self):
        self.marker_list = []
        print("Please start ARUCO node within 15 sec")

        time.sleep(15)
        msg = rospy.wait_for_message("/aruco_marker_publisher/markers", MarkerArray, timeout=None)
        self.print_msg(msg)
        self.msg = msg

        print("Message received, please shutdown node")
        time.sleep(15)

    def print_msg(self, msg):
        for marker in msg.markers:
            self.marker_list.append({'id': marker.id, 'x': marker.pose.pose.position.x, 'y': marker.pose.pose.position.y, 'z': marker.pose.pose.position.z})
            print("Marker ID: ", marker.id)
            print("X: ", marker.pose.pose.position.x)
            print("Y: ", marker.pose.pose.position.y)
            print("Z: ", marker.pose.pose.position.z)

        print("Marker List: ")
        print(self.marker_list)

    def calc_arm_to_obj(self, id):
        [x, y, z] = self.get_arm_transform()
        print("arm coords ")
        print("X: ", x)
        print("Y: ", y)
        print("Z: ", z)

    def get_arm_transform(self):
        # if self.tf.frameExists("/base_link") and self.tf.frameExists("/gripper_right_grasping_frame"):
        # t = self.tf_listener.getLatestCommonTime("/base_link", "/gripper_right_grasping_frame")
        p1 = PoseStamped()
        p1.header.frame_id = "gripper_right_grasping_frame"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener.transformPose("/base_footprint", p1)
        # print ("Position of the fingertip in the robot base:")
        # print (p_in_base)
        return p_in_base.pose.position.x, p_in_base.pose.position.y, p_in_base.pose.position.z
        

if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('marker_man', anonymous=True)


    marker = marker_manager()
    marker.get_markers()
                


        

    
