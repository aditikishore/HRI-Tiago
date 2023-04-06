#!/usr/bin/env/python

import sys
import copy

import rospy

import moveit_commander
import moveit_msgs.msg

from geometry_msgs.msg import Twist
from math import pi
from std_msgs.msg import Bool

import move_base

from moveit_commander.conversions import pose_to_list



def init_ros():
    
    rospy.init_node('hri_node', anonymous=False)
    

if __name__=='__main__':
    init_ros()

    base = move_base.robot_base()
    base.move_from_init_to_home()
    base.move_from_home_to_inv()
    base.move_from_inv_to_tar()
    base.move_from_tar_to_home()



