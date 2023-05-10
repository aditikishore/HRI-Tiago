"""
Move-Body - ROSPY code for moving the head and the torso of the robot for ARUCO marker
detection on the object table.

In this file, we have created a class robot_body which has functions that can be executed to 
tell Tiago to move it's head and/or it's torso. We used control_msgs, trajectory_msgs and 
geometry_msgs for these tasks.

This code is used in the main file, wherein we create an object of the class robot_body
and use this object to perform various movement for the Tiago. 

Usage:
    1. Import the necessary modules:
        import rospy
        import time

        import actionlib
        from geometry_msgs.msg import PointStamped
        from control_msgs.msg import PointHeadAction, PointHeadGoal
        from pal_common_msgs.msg import DisableGoal, DisableAction
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from play_motion_msgs.msg import PlayMotionActionGoal
    
    2. Create an instance of the robot_body class (in main.py):
        body = move_body.robot_body()
        
    3. Call the various functions of the class using the created instance in main.py:
        body.raise_torso()
        body.look_at_inv()
        body.center_torso()
        body.look_straight()

    4. Run the node using the main() function in main.py:
        if __name__ == '__main__':
            HRI_Script()
"""

#!/usr/bin/env/python

import rospy
import time

import actionlib
from geometry_msgs.msg import PointStamped
from control_msgs.msg import PointHeadAction, PointHeadGoal
from pal_common_msgs.msg import DisableGoal, DisableAction
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from play_motion_msgs.msg import PlayMotionActionGoal

"""
Class robot_body - We created a class with some functions and variables that belong to the
Tiago Robot for easier computation and data transfer to the Tiago while performing our intended 
task.
"""

class robot_body:

    """
    Initialise function for the Robot Body Joints.

    Sets the joint to predefined values that we have found out. Also initialises the reference frames
    and clients and publishers that we will use for these tasks.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

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
        
        # left arm
        self.left_arm_jt = JointTrajectory()
        self.left_arm_jt.joint_names = ["arm_left_1_joint", "arm_left_2_joint", "arm_left_3_joint",
                                         "arm_left_4_joint", "arm_left_5_joint", "arm_left_6_joint", "arm_left_7_joint"]
        self.left_arm_jtp = JointTrajectoryPoint()
        self.left_arm_jtp.positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.left_arm_jtp.time_from_start = rospy.Duration(3)
        self.left_arm_jt.points.append(self.left_arm_jtp)

        self.left_arm_ext_pos = [1.60, 0.6, 0.0, 0.65, -1.57, 0.0, 0.0]

        self.left_arm_pub = rospy.Publisher(
            '/arm_left_controller/command', JointTrajectory, queue_size=1)
        
        print('Initialised body')
        
    #methods
    """
    Function to instruct tiago to look at a pre-defined point.

    This function tells the tiago's head to look in a particular direction towards
    a pre-defined point, irrespective of the current field of view of the tiago.
    It will cause the head of the tiago to move accordingly.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            it publishes the value directly to the respective node.
    """

    def look_at_inv(self):
        print("look_to_point")
        self.head_goal.target = self.inv_point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    """
    Function to instruct tiago to look at straight.

    This function tells the tiago's head to look straight, irrespective of the current field of view of the tiago.
    The straight direction is considered a neutral point for the tiago to look at.
    It will cause the head of the tiago to move accordingly.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            it publishes the value directly to the respective node.
    """

    def look_straight(self):
        print("look_to_point")
        self.head_goal.target = self.neutral_point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    """
    Function to instruct tiago to look at a new point.

    This function tells the tiago's head to look in a direction defined by the new
    coordinates passed in the function. 
    It first generated a point stamp to move to and then it will cause the head 
    of the tiago to move accordingly.

        Args:
            self: passes the object calling the function as a parameter.
            x: the x-coordinate of the point to look at.
            y: the y-coordinate of the point to look at.
            z: the z-coordinate of the point to look at.
        Returns:
            no data is returned in this function.
            it publishes the value directly to the respective node.
    """

    def look_to_point(self, x, y, z):
        point = self.create_point_stamped(x, y, z)
        self.head_goal.target = point
        self.head_client.send_goal(self.head_goal)
        # self.head_client.wait_for_result()

    """
    Function to make the TiaGo look towards the table.

    This function tells the tiago's head to move downwards towards the table. It is
    done so that we can then call the camera of the TiaGo to locate where the objects are placed.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def look_down(self):
        self.move_head(self.look_down_j)

    """
    Function to make the TiaGo look further down towards the table.

    This function tells the tiago's head to move downwards towards the table. It was created
    as a cautionary function incase the look_down function fails to detect any markers on the
    table in the first try.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def look_down_more(self):
        self.move_head(self.look_down_more_j)

    """
    Function to move the head to the speficified joint position.

    This function tells the Tiago's head to move to a specific joint position
    as passed in the function argument. It directly publishes this joint position
    to the head_pub, which is the node for head movement of the Tiago.

        Args:
            self: passes the object calling the function as a parameter.
            j: the joint position to which the tiago's head should move.
        Returns:
            no data is returned in this function.
    """

    def move_head(self, j):
        print('move_head')
        self.head_jtp.positions = j
        self.head_jt.points[0] = self.head_jtp
        self.head_pub.publish(self.head_jt)
        time.sleep(5)

    """
    Function to control the alive engine of the Tiago.

    This function enables of disables the alive engine of the tiago.
    If the command is "enable", it will start the head_manager node. If not, it
    will stop the head_manager node. The head_manager node is used to communicate
    head movement in the TiaGo.

        Args:
            self: passes the object calling the function as a parameter.
            command: string input that tells the tiago whether to start or stop the 
                     head_manager node.
        Returns:
            no data is returned in this function.
    """

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

    """
    Function to raise the Torso.

    This function sends a value to the move_torso function of this code that instructs the 
    Tiago to raise the torso to some height. This height was determined using experimental
    conditions.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            it calls another function to pass information to the Node of the Tiago.

    """

    def raise_torso(self):
        self.move_torso(0.34)

    """
    Function to center the Torso.

    This function sends a value to the move_torso function of this code that instructs the 
    Tiago to move the torso to the pre-defined center. This height was determined using experimental
    conditions.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            it calls another function to pass information to the Node of the Tiago.
    """

    def center_torso(self):
        self.move_torso(0.20)

    """
    Function to lower the Torso.

    This function sends a value to the move_torso function of this code that instructs the 
    Tiago to lower the torso to the base height. 

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            it calls another function to pass information to the Node of the Tiago.
    """
    def lower_torso(self):
        self.move_torso(0.04)

    """
    Function to move the torso of the Tiago to some height.

    This function moves the torso of the Tiago to the given value. It publishes the value onto
    the node which actually moves the tiago's torso. It contains an input joint position that
    determines what position the joint (Torso) should move to.

        args:
            self: passes the object calling the function as a parameter.
            p: the value to which the Tiago's Torso should move. These are pre-defined by us
                using experimental condition values.
        Returns:
            no data is returned in this function.
            it publishes the value directly to the respective node.

    """

    def move_torso(self, p):
        print('move_torso')
        self.torso_jtp.positions = [p]
        self.torso_jt.points[0] = self.torso_jtp
        self.torso_pub.publish(self.torso_jt)
        time.sleep(5)

    """
    Function to extend the right arm of the Tiago.
    
    This function tells the robot to extend the right arm fully ahead. It is used when we want to
    slide an object between its gripper.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def extend_right_arm(self):
        self.play_motion('offer_right')
        time.sleep(8)
        self.center_torso()
        time.sleep(3)
        self.move_right_arm(self.right_arm_ext_pos)
        time.sleep(5)

    """
    Function to control the right arm of the Tiago.
    
    This function tells the robot to extend the right arm to a specific joint location. The joint
    location is passed to the function as a parameter.
    
        Args:
            self: passes the object calling the function as a parameter.
            j: the joint position to which the Tiago's right arm should reach.
        Returns:
            no data is returned in this function.
    """

    def move_right_arm(self, j):
        print('move_right_arm')
        self.right_arm_jtp.positions = j
        self.right_arm_jt.points[0] = self.right_arm_jtp
        self.right_arm_pub.publish(self.right_arm_jt)
        time.sleep(5)

    """
    Function to retract the right arm of the Tiago.
    
    This function tells the robot to retract (get back) the right arm to it's body. It is used
    after we hold an object between the gripper of the arm.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """    
    def retract_right_arm(self):
        self.play_motion('home_right')
        time.sleep(15)

    """
    Function to extend the left arm of the Tiago.
    
    This function tells the robot to extend the left arm fully ahead. It is used when we want to
    slide an object between its gripper.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def extend_left_arm(self):
        self.play_motion('offer_left')
        time.sleep(8)
        self.center_torso()
        time.sleep(3)
        self.move_left_arm(self.left_arm_ext_pos)
        time.sleep(5)

    """
    Function to control the left arm of the Tiago.
    
    This function tells the robot to extend the left arm to a specific joint location. The joint
    location is passed to the function as a parameter.
    
        Args:
            self: passes the object calling the function as a parameter.
            j: the joint position to which the Tiago's left arm should reach.
        Returns:
            no data is returned in this function.
    """
    def move_left_arm(self, j):
        print('move_left_arm')
        self.left_arm_jtp.positions = j
        self.left_arm_jt.points[0] = self.left_arm_jtp
        self.left_arm_pub.publish(self.left_arm_jt)
        time.sleep(5)
        
    """
    Function to retract the left arm of the Tiago.
    
    This function tells the robot to retract (get back) the left arm to it's body. It is used
    after we hold an object between the gripper of the arm.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def retract_left_arm(self):
        self.play_motion('home_left')
        time.sleep(15)

    """
    Function to CLOSE the gripper on the right arm.
    
    This function tells the robot to close the gripper on the right arm of the Tiago. It is used
    to hold the object on the table.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def close_gripper_right(self):
        self.play_motion('close_right')
        time.sleep(5)

    """
    Function to OPEN the gripper on the right arm.
    
    This function tells the robot to close the gripper on the right arm of the Tiago. It is used
    to hold the object on the table.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def open_gripper_right(self):
        self.play_motion('open_right')
        time.sleep(5)

    """
    Function to CLOSE the gripper on the left arm.
    
    This function tells the robot to close the gripper on the left arm of the Tiago. It is used
    to hold the object on the table.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def close_gripper_left(self):
        self.play_motion('close_left')
        time.sleep(5)

    """
    Function to OPEN the gripper on the left arm.
    
    This function tells the robot to close the gripper on the left arm of the Tiago. It is used
    to hold the object on the table.
    It calls another function in this code to do the actuation using pre-defined arm movements.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def open_gripper_left(self):
        self.play_motion('open_left')
        time.sleep(5)

    """
    Function to publish commands onto the Tiago's Arm.
    
    This function tells the robot which arm movement to do. It can choose from pre-defined motions
    such as 'open_left' and 'open_right' which are gripper movements. 
    
        Args:
            self: passes the object calling the function as a parameter.
            motion: the pre-defined motion that we want the TiaGo to follow.
        Returns:
            no data is returned in this function.
    """

    def play_motion(self, motion):
        print('play_motion: ', motion)
        self.play_motion_goal.goal.motion_name = motion
        self.play_motion_pub.publish(self.play_motion_goal)
        time.sleep(5)

    """
    Function to create a point stamp (point goal) for the tiago to look at. 
    
    This function is called by the function look_to_point to set the new goal point 
    of the Tiago's head joint. 
    It will set the x, y and z coordinates of the goal point to the point variable of the
    Tiago which will be then used as a goal state for the head to move to.

        Args:
            self: passes the object calling the function as a parameter.
            x: the x-coordinate of the goal point.
            y: the y-coordinate of the goal point.
            z: the z-coordinate of the goal point.

        Returns:
            point: the new variable point which contains the goal x, y and z coordinate values
                    that can be accessed by the Tiago's Node as required.
    """

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
