#!/usr/bin/env/python

import sys

import rospy

import move_base

from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
import actionlib
from geometry_msgs.msg import PointStamped



def init_ros():
    
    rospy.init_node('hri_node', anonymous=False)
    

if __name__=='__main__':
    init_ros()

    base = move_base.robot_base()
    base.move_from_init_to_home()
    #base.move_from_home_to_inv()
    #base.move_from_inv_to_tar()
    #base.move_from_tar_to_home()

    camera_frame = "/xtion_rgb_optical_frame"
    base_frame = "base_link"

    point = PointStamped()
    point.header.frame_id = base_frame
    point.header.stamp = rospy.Time.now()
    point.point.x = 2       #distance outward
    point.point.y = 0       #distance left
    point.point.z = 0.3     #distance up

    goal = PointHeadGoal()
    goal.pointing_frame = camera_frame
    goal.pointing_axis.x = 0.0
    goal.pointing_axis.y = 0.0
    goal.pointing_axis.z = 1.0
    goal.min_duration = rospy.Duration(1.0)
    goal.max_velocity = 0.25
    goal.target = point

    client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
    client.wait_for_server()

    client.send_goal(goal)
    client.wait_for_result()