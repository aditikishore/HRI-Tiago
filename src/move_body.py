#!/usr/bin/env/python

import rospy
import time

import actionlib
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class robot_body:
    def __init__(self):

        # head stuff
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.base_frame = "base_link"

        self.inv_point = self.create_point_stamped(2, 0, 0.3)
        self.neutral_point = self.create_point_stamped(20, 0, 1.6)

        self.head_goal = PointHeadGoal()
        self.head_goal.pointing_frame = self.camera_frame
        self.head_goal.pointing_axis.x = 0.0
        self.head_goal.pointing_axis.y = 0.0
        self.head_goal.pointing_axis.z = 1.0
        self.head_goal.min_duration = rospy.Duration(1.0)
        self.head_goal.max_velocity = 0.25

        self.head_goal.target = self.inv_point

        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()

        # torso stuff
        self.torso_jt = JointTrajectory()
        self.torso_jt.joint_names = ['torso_lift_joint']
        self.torso_jtp = JointTrajectoryPoint()
        self.torso_jtp.positions = [0.0]
        self.torso_jtp.time_from_start = rospy.Duration(3)
        self.torso_jt.points.append(self.torso_jtp)

        self.torso_pub = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)

    def look_at_inv(self):
        print("look_to_point")
        self.head_goal.target = self.inv_point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    def look_straight(self):
        print("look_to_point")
        self.head_goal.target = self.neutral_point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    def look_to_point(self, x, y, z):
        point = self.create_point_stamped(x, y, z)
        self.head_goal.target = point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    def raise_torso(self):
        self.move_torso(0.34)

    def center_torso(self):
        self.move_torso(0.2)

    def lower_torso(self):
        self.move_torso(0.0)

    def move_torso(self, p):
        print('move_torso')
        self.torso_jtp.positions = [p]
        self.torso_jt.points[0] = self.torso_jtp
        self.torso_pub.publish(self.torso_jt)
        time.sleep(5)

    def create_point_stamped(self, x, y, z):
        now = 0
        while not now:  # wait for clock message to publish
            now = rospy.Time.now()

        point = PointStamped()
        point.header.frame_id = self.base_frame
        point.header.stamp = rospy.Time.now()
        point.point.x = x  # distance outward
        point.point.y = y  # distance left
        point.point.z = z  # distance up
        return point
