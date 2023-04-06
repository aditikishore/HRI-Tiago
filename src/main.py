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

def init_ros_publishers():
    
    base_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist,queue_size=1)


    return base_vel_publisher

def base_command(lin,ang):
    move_cmd = Twist()
    move_cmd.linear.x = lin[0]
    move_cmd.linear.y = lin[1]
    move_cmd.linear.z = lin[2] # m/s
    move_cmd.angular.x = ang[0]
    move_cmd.angular.y = ang[1]
    move_cmd.angular.z = ang[2]

    return move_cmd




if __name__=='__main__':
    init_ros()
    
    #p,joy=init_ros_publishers()

    #joy_cmd = Bool()
    #joy_cmd.data = False

    #fwd=base_command([0.2,0.0,0.0],[0.0,0.0,0.0])
    #stop=base_command([0.0,0.0,0.0],[0.0,0.0,0.0])
    #left=base_command([0.0,0.0,0.0],[0.0,0.0,-0.2])
    
    #now = rospy.Time.now()
    #rate = rospy.Rate(60)
    
    #while rospy.Time.now() < now + rospy.Duration.from_sec(7):

        #joy.publish(joy_cmd)
        #p.publish(fwd)
        #rate.sleep() 

    #p.publish(stop)

    #now = rospy.Time.now()

    #while rospy.Time.now() < now + rospy.Duration.from_sec(7.85):
        #joy.publish(joy_cmd)
        #p.publish(left)
        #rate.sleep() 

    #p.publish(stop)

    #now = rospy.Time.now()

    #while rospy.Time.now() < now + rospy.Duration.from_sec(7):

        #joy.publish(joy_cmd)
        #p.publish(fwd)
        #rate.sleep()  

    #p.publish(stop)



