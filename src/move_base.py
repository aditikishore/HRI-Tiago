#!/usr/bin/env/python

import rospy

from geometry_msgs.msg import Twist
from math import pi

class robot_base:
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.current_pos = "init"


        self.lin_vel = 0.2
        self.ang_vel = 0.2

        self.half_base_length = 0.36
        self.half_base_width = 0.25


        self.cmd_w = self.generate_twist([self.lin_vel,0.0,0.0],[0.0,0.0,0.0])
        self.cmd_s = self.generate_twist([-self.lin_vel,0.0,0.0],[0.0,0.0,0.0])
        self.cmd_a = self.generate_twist([0.0,self.lin_vel,0.0],[0.0,0.0,0.0])
        self.cmd_d = self.generate_twist([0.0,-self.lin_vel,0.0],[0.0,0.0,0.0])
        self.cmd_l = self.generate_twist([0.0,0.0,0.0],[0.0,0.0,self.ang_vel])
        self.cmd_r = self.generate_twist([0.0,0.0,0.0],[0.0,0.0,-self.ang_vel])
        self.cmd_stop = self.generate_twist([0.0,0.0,0.0],[0.0,0.0,0.0])


        self.robot_base_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel',Twist,queue_size=1)

    def move_from_init_to_home(self):   #robot moves to home position and faces user
        self.move_distance(1.9 + self.half_base_length, self.cmd_w)
        self.move_distance(1.75 - self.half_base_width, self.cmd_d)
        self.move_angle(60, self.cmd_l)
        self.robot_base_publisher.publish(self.cmd_stop)
        self.current_pos = "home"

    def move_from_home_to_inv(self):    #robot moves close to inventory table. ready to pick
        self.move_angle(150, self.cmd_r)
        self.move_distance(1.15 - self.half_base_length, self.cmd_w)
        self.robot_base_publisher.publish(self.cmd_stop)  
        self.current_pos = "inv"

    def move_from_inv_to_tar(self):     #robot moves close to target toable, ready to place
        self.move_distance(1.15 + 1.0 - self.half_base_length, self.cmd_s)
        self.move_angle(90, self.cmd_l)
        self.move_distance(0.75 - self.half_base_width - 0.1, self.cmd_a)
        self.move_distance(0.6 - self.half_base_length - 0.1, self.cmd_w)
        self.robot_base_publisher.publish(self.cmd_stop)
        self.current_pos = "tar"

    def move_from_tar_to_home(self):    #robot moves back to home, ready for another command
        self.move_distance(0.6 - self.half_base_length - 0.1, self.cmd_s)
        self.move_distance(1.75 - self.half_base_width - 0.1, self.cmd_d)
        self.move_angle(60, self.cmd_l) 
        self.robot_base_publisher.publish(self.cmd_stop)
        self.current_pos = "home"

    def move_distance(self, distance, command): #meters
        duration = distance/self.lin_vel
        print("move_dist")

        now = 0
        while not now:      #wait for clock message to publish
            now = rospy.Time.now()
        end_time = now + rospy.Duration.from_sec(duration)
        
        while rospy.Time.now() < end_time:
            self.robot_base_publisher.publish(command)
            self.rate.sleep()

    def move_angle(self, angle, command): #degrees
        duration = (pi/180)*(angle/self.ang_vel)
        print("move_ang")

        now = 0
        while not now:
            now = rospy.Time.now()

        end_time = now + rospy.Duration.from_sec(duration)
        while rospy.Time.now() < end_time:
            self.robot_base_publisher.publish(command)
            self.rate.sleep()

    def generate_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]

        return twist