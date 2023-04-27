#!/usr/bin/env/python

import rospy

from geometry_msgs.msg import Twist
import math


class robot_base:
    def __init__(self):
        self.rate = rospy.Rate(60)
        self.current_pos = "init"

        self.lin_vel = 0.2
        self.ang_vel = 0.4

        self.half_base_length = 0.36
        self.half_base_width = 0.25

        self.X_pos = self.half_base_length
        self.Y_pos = self.half_base_width
        self.ori_pos = math.pi


        #WAYPOINTS
        self.first_turn_x = -1.0

        self.home_x = -1.8
        self.home_y = 1.3

        self.inv_y = 2.3

        self.tar_x = -2.2
        self.tar_y = 0.5
        ##########


        self.cmd_w = self.generate_twist(
            [self.lin_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.cmd_s = self.generate_twist(
            [-self.lin_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.cmd_a = self.generate_twist(
            [0.0, self.lin_vel, 0.0], [0.0, 0.0, 0.0])
        self.cmd_d = self.generate_twist(
            [0.0, -self.lin_vel, 0.0], [0.0, 0.0, 0.0])
        self.cmd_l = self.generate_twist(
            [0.0, 0.0, 0.0], [0.0, 0.0, self.ang_vel])
        self.cmd_r = self.generate_twist(
            [0.0, 0.0, 0.0], [0.0, 0.0, -self.ang_vel])
        self.cmd_stop = self.generate_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        self.robot_base_publisher = rospy.Publisher(
            '/mobile_base_controller/cmd_vel', Twist, queue_size=1)

    # robot moves to home position and faces user
    def move_from_init_to_home(self):
        self.move_to_point(self.first_turn_x, self.half_base_width, 180)
        self.move_to_point(self.home_x, self.home_y, -95)
        self.current_pos = "home"

    # robot moves close to inventory table. ready to pick
    def move_from_home_to_inv(self):
        self.move_to_point(self.home_x, self.inv_y-self.half_base_length, 90)
        self.current_pos = "inv"

    # robot moves close to target table, ready to place
    def move_from_inv_to_tar(self):
        self.move_to_point(self.tar_x + self.half_base_width, self.home_y, -90)
        self.move_to_point(self.tar_x + self.half_base_width, self.tar_y + self.half_base_length, -90)
        self.current_pos = "tar"

    # robot moves back to home, ready for another command
    def move_from_tar_to_home(self):
        self.move_to_point(self.home_x, self.home_y, -95)
        self.current_pos = "home"

    # move from wherever the robot is to a point and rotate to the ori. keep ori between -180 and 180
    def move_to_point(self, x, y, ori):
        delta_x = x - self.X_pos
        delta_y = y - self.Y_pos

        angle_to = math.atan2(delta_y, delta_x)
        angle = self.limit_angle_to_range(angle_to - self.ori_pos)

        if (angle < 0):
            self.move_angle(-angle * (180/math.pi), self.cmd_r)
        else:
            self.move_angle(angle * (180/math.pi), self.cmd_l)

        self.ori_pos = angle_to

        distance = math.sqrt((delta_x)**2 + (delta_y)**2)
        self.move_distance(distance, self.cmd_w)
        self.X_pos = x
        self.Y_pos = y

        angle = self.limit_angle_to_range(ori * (math.pi/180) - self.ori_pos)

        if (angle < 0):
            self.move_angle(-angle * (180/math.pi), self.cmd_r)
        else:
            self.move_angle(angle * (180/math.pi), self.cmd_l)

        self.ori_pos = ori * (math.pi/180)

        print("Robot now at:", self.X_pos, self.Y_pos)

    def move_distance(self, distance, command):  # meters
        duration = distance/self.lin_vel
        print("move_dist")

        now = 0
        while not now:  # wait for clock message to publish
            now = rospy.Time.now()
        end_time = now + rospy.Duration.from_sec(duration)

        #print("expected duration:", rospy.Duration.from_sec(duration))

        while rospy.Time.now() < end_time:
            self.robot_base_publisher.publish(command)
            self.rate.sleep()

        #print("actual duration:", (rospy.Time.now()-now))
        
        self.robot_base_publisher.publish(self.cmd_stop)

    def move_angle(self, angle, command):  # degrees
        angle = abs(angle) + 0.5 #friction
        duration = (math.pi/180)*(angle/self.ang_vel)
        print("move_ang")

        now = 0
        while not now:
            now = rospy.Time.now()

        end_time = now + rospy.Duration.from_sec(duration)
        while rospy.Time.now() < end_time:
            self.robot_base_publisher.publish(command)
            self.rate.sleep()

        self.robot_base_publisher.publish(self.cmd_stop)

    def generate_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]

        return twist

    def limit_angle_to_range(self, angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
        if angle < -math.pi:
            angle = angle + 2*math.pi
        return angle
