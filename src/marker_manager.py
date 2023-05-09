#!/usr/bin/env/python

import rospy
import time

from geometry_msgs.msg import Pose, PoseStamped
from aruco_msgs.msg import Marker, MarkerArray
import subprocess
import tf

    

class marker_manager:
    def __init__(self):

        self.marker_dict = dict()
        self.marker_dict[9] = {'x': 1.0, 'y': -0.6, 'z': 1}
        self.marker_dict[10] = {'x': 0.8, 'y': -0.3, 'z': 1}
        self.marker_dict[11] = {'x': 0.8, 'y': 0.0, 'z': 1}
        self.marker_dict[8] = {'x': 0.8, 'y': 0.3, 'z': 1}
        self.marker_dict[12] = {'x': 1.0, 'y': 0.6, 'z': 1}


        self.msg = MarkerArray()
        self.tf_listener = tf.TransformListener()
        print('Initialised marker manager')


    def get_markers(self):
        self.marker_dict = dict()
        print("Please start ARUCO node")
        # subprocess.run(["rosrun", "aruco_ros", "marker_publish_node"])
        time.sleep(2)
        msg = rospy.wait_for_message("/aruco_marker_publisher/markers", MarkerArray, timeout=None)
        self.store_markers(msg)
        self.msg = msg
        # subprocess.run(["rosnode", "kill", "/aruco_marker_publisher"])
        print("Message received, please shutdown node within 5 sec")
        time.sleep(5)
        return self.marker_dict

    def store_markers(self, msg):
        for marker in msg.markers:
            self.marker_dict[marker.id] = {'x': marker.pose.pose.position.x, 'y': marker.pose.pose.position.y, 'z': marker.pose.pose.position.z}
            print("Got marker ID: ", marker.id)

        # print("Stored marker List: ")
        # print(self.marker_dict)

    def calc_arm_to_obj(self, id):
        [x, y, z] = self.get_arm_transform()
        # print("arm coords ")
        # print("X: ", x)
        # print("Y: ", y)
        # print("Z: ", z)
        X_off  = self.marker_dict[id]['x'] - x
        Y_off = self.marker_dict[id]['y'] - y
        
        return X_off, Y_off

    def get_arm_transform(self):
        p1 = PoseStamped()
        p1.header.frame_id = "gripper_right_grasping_frame"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener.transformPose("/base_footprint", p1)
        print (p_in_base)
        return p_in_base.pose.position.x, p_in_base.pose.position.y, p_in_base.pose.position.z
        

if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('marker_man', anonymous=True)


    marker = marker_manager()
    marker.get_markers()
                


        

    
