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
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    
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
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

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

        # Set the reference frames for the head
        self.camera_frame = "/xtion_rgb_optical_frame"
        self.base_frame = "base_link"

        # Set the pre-defined points for the head to move to. 
        self.inv_point = self.create_point_stamped(2, 0, 0.3)
        self.neutral_point = self.create_point_stamped(20, 0, 1.6)

        # Create an object of the goal position for the head and give it x and y coordinates
        # along with the speed for going to those coordinates
        self.head_goal = PointHeadGoal()
        self.head_goal.pointing_frame = self.camera_frame
        self.head_goal.pointing_axis.x = 0.0
        self.head_goal.pointing_axis.y = 0.0
        self.head_goal.pointing_axis.z = 1.0
        self.head_goal.min_duration = rospy.Duration(1.0)
        self.head_goal.max_velocity = 0.25

        # Set the target point for the head
        self.head_goal.target = self.inv_point

        # Publish all these variables onto the ROS Client Node for the Head
        self.head_client = actionlib.SimpleActionClient(
            '/head_controller/point_head_action', PointHeadAction)
        self.head_client.wait_for_server()

        # Set the names and create an object for the Torso of the tiago
        self.torso_jt = JointTrajectory()
        self.torso_jt.joint_names = ['torso_lift_joint']
        self.torso_jtp = JointTrajectoryPoint()
        self.torso_jtp.positions = [0.0]
        self.torso_jtp.time_from_start = rospy.Duration(3)
        self.torso_jt.points.append(self.torso_jtp)

        # Publish this information to the ROS Publisher Node for the Torso
        self.torso_pub = rospy.Publisher(
            '/torso_controller/command', JointTrajectory, queue_size=1)
        
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
        self.move_torso(0.2)

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
        self.move_torso(0.0)

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
        point.point.x = x  # distance outward
        point.point.y = y  # distance left
        point.point.z = z  # distance up
        return point
