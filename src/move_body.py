#!/usr/bin/env/python

import rospy
import time

import actionlib
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_common_msgs.msg import DisableGoal, DisableAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionActionGoal


class robot_body:
    def __init__(self):

        # head look to point
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.base_frame = "base_link"

        self.inv_point = self.create_point_stamped(1.5, 0, 0.3)
        self.neutral_point = self.create_point_stamped(20, 0, 1.6)
        self.look_down_j = [0.0, -0.6]
        self.look_down_more_j = [0.0, -0.8]

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

        self.head_mgr_client = actionlib.SimpleActionClient(
            '/pal_head_manager/disable', DisableAction)
        
        # head stuff
        self.head_jt = JointTrajectory()
        self.head_jt.joint_names = ['head_1_joint', 'head_2_joint']
        self.head_jtp = JointTrajectoryPoint()
        self.head_jtp.positions = [0.0, 0.0]
        self.head_jtp.time_from_start = rospy.Duration(2)
        self.head_jt.points.append(self.head_jtp)

        self.head_pub = rospy.Publisher(
            '/head_controller/command', JointTrajectory, queue_size=1)

        # torso stuff
        self.torso_jt = JointTrajectory()
        self.torso_jt.joint_names = ['torso_lift_joint']
        self.torso_jtp = JointTrajectoryPoint()
        self.torso_jtp.positions = [0.0]
        self.torso_jtp.time_from_start = rospy.Duration(3)
        self.torso_jt.points.append(self.torso_jtp)

        self.torso_pub = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)

        # play motion
        self.play_motion_goal = PlayMotionActionGoal()
        self.play_motion_goal.header.stamp = rospy.Time.now()
        self.play_motion_goal.goal.skip_planning = False
        self.play_motion_goal.goal.priority = 0

        self.play_motion_pub = rospy.Publisher(
            '/play_motion/goal', PlayMotionActionGoal, queue_size=1)

        # right arm
        self.right_arm_jt = JointTrajectory()
        self.right_arm_jt.joint_names = ["arm_right_1_joint", "arm_right_2_joint", "arm_right_3_joint",
                                         "arm_right_4_joint", "arm_right_5_joint", "arm_right_6_joint", "arm_right_7_joint"]
        self.right_arm_jtp = JointTrajectoryPoint()
        self.right_arm_jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_arm_jtp.time_from_start = rospy.Duration(3)
        self.right_arm_jt.points.append(self.right_arm_jtp)

        self.right_arm_ext_pos = [1.60, 0.6, 0.0, 0.65, -1.57, 0.0, 0.0]

        self.right_arm_pub = rospy.Publisher(
            '/arm_right_controller/command', JointTrajectory, queue_size=1)
        
        print('Initialised body')
        
    #methods

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
        
    def look_down(self):
        self.move_head(self.look_down_j)

    def look_down_more(self):
        self.move_head(self.look_down_more_j)

    def move_head(self, j):
        print('move_head')
        self.head_jtp.positions = j
        self.head_jt.points[0] = self.head_jtp
        self.head_pub.publish(self.head_jt)
        time.sleep(5)

    def head_mgr(self, command):

        if (not self.head_mgr_client.wait_for_server(
                timeout=rospy.Duration(2.0))):
            print("failed to connect to head manager node")
            return

        if (command == "enable"):
            print("enabling head manager.")
            self.head_mgr_client.cancel_goal()

        elif (command == "disable"):
            action = DisableGoal()
            print("disabling head manager")
            action.duration = 0.0
            self.head_mgr_client.send_goal(action)

    def raise_torso(self):
        self.move_torso(0.34)

    def center_torso(self):
        self.move_torso(0.17)

    def lower_torso(self):
        self.move_torso(0.08)

    def move_torso(self, p):
        print('move_torso')
        self.torso_jtp.positions = [p]
        self.torso_jt.points[0] = self.torso_jtp
        self.torso_pub.publish(self.torso_jt)
        time.sleep(5)

    def extend_right_arm(self):
        self.play_motion('offer_right')
        time.sleep(8)
        self.center_torso()
        time.sleep(3)
        self.move_right_arm(self.right_arm_ext_pos)
        time.sleep(5)

    def move_right_arm(self, j):
        print('move_right_arm')
        self.right_arm_jtp.positions = j
        self.right_arm_jt.points[0] = self.right_arm_jtp
        self.right_arm_pub.publish(self.right_arm_jt)
        time.sleep(5)
        
    def retract_right_arm(self):
        self.play_motion('home_right')
        time.sleep(15)

    def close_gripper_right(self):
        self.play_motion('close_right')
        time.sleep(5)

    def open_gripper_right(self):
        self.play_motion('open_right')
        time.sleep(5)

    def play_motion(self, motion):
        print('play_motion: ', motion)
        self.play_motion_goal.goal.motion_name = motion
        self.play_motion_pub.publish(self.play_motion_goal)
        time.sleep(5)

    def create_point_stamped(self, x, y, z):
        now = 0
        while not now:  # wait for clock message to publish
            now = rospy.Time.now()

        point = PointStamped()
        point.header.frame_id = self.base_frame
        point.header.stamp = rospy.Time.now()
        point.point.x = x  # distance forward
        point.point.y = y  # distance left
        point.point.z = z  # distance up
        return point
