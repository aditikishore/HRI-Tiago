"""
Move-Base - ROSPY code for moving the base of the TiaGo to required locations.

In this file, we have created a class robot_base which has functions that can be executed to 
tell Tiago to move it's body to various positions that we decided as home, intermediate home,
inventory table and target table.

This code is used in the main file, wherein we create an object of the class robot_base
and use this object to perform various movement for the Tiago. 

Usage:
    1. Import the necessary modules:
        import rospy

        from geometry_msgs.msg import Twist
        import math
    
    2. Create an instance of the robot_body class (in main.py):
        base = move_base.robot_base()
        
    3. Call the various functions of the class using the created instance in main.py:
        base.move_from_inv_to_arm()
        base.move_from_inv_to_tar()

    4. Run the node using the main() function in main.py:
        if __name__ == '__main__':
            HRI_Script()
"""

#!/usr/bin/env/python

import rospy

from geometry_msgs.msg import Twist
import math

"""
Class robot_base - We created a class with some functions and variables that belong to the
Tiago Robot for easier computation and data transfer to the Tiago while performing our intended 
task.

"""
class robot_base:

    """
    Initialise function for the Robot Body.

    Sets the linear and angular velocities that the robot will use during traversal. We even coded in 
    some twists to use with the keyboard for the robot.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
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

        self.obj_x = 0.0
        self.obj_y = 0.0

        #WAYPOINTS
        self.first_turn_x = -1.0

        self.home_x = -1.7
        self.home_y = 1.3

        self.inv_y = 2.1
        
        self.inv_arm_y = 1.8

        self.tar_x = -2.2
        self.tar_y = 0.65
        ##########


        self.cmd_w = self.generate_twist(
            [self.lin_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.cmd_s = self.generate_twist(
            [-self.lin_vel, 0.0, 0.0], [0.0, 0.0, 0.0])
        self.cmd_a = self.generate_twist(
            [0.0, self.lin_vel * 1.2, 0.0], [0.0, 0.0, 0.0])
        self.cmd_d = self.generate_twist(
            [0.0, -self.lin_vel * 1.2, 0.0], [0.0, 0.0, 0.0])
        self.cmd_l = self.generate_twist(
            [0.0, 0.0, 0.0], [0.0, 0.0, self.ang_vel])
        self.cmd_r = self.generate_twist(
            [0.0, 0.0, 0.0], [0.0, 0.0, -self.ang_vel])
        self.cmd_stop = self.generate_twist([0.0, 0.0, 0.0], [0.0, 0.0, 0.0])

        self.robot_base_publisher = rospy.Publisher(
            '/mobile_base_controller/cmd_vel', Twist, queue_size=1)
        
        print('Initialised base')

    """
    Function to move the tiago from the initial position to an intermediate home.

    This function makes the tiago move from it's starting location to a new home location
    that we decided using experimental conditions.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_init_to_home(self):
        self.move_to_point(self.first_turn_x, self.half_base_width, 180)
        self.move_to_point(self.home_x, self.home_y, -95)
        self.current_pos = "home"

    """
    Function to move the tiago from the new home position to the inventory table

    This function makes the tiago move from it's new home location to the inventory table for picking up objects.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_home_to_inv(self):
        self.move_to_point(self.home_x, self.inv_y-self.half_base_length, 90)
        self.current_pos = "inv"

    """
    Function to move the tiago from the inventory table a little behind.

    This function makes the tiago move from the inventory table a little behind so that it can extend its arm to grab
    and object on the table comfortably without colliding anywhere.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_inv_to_arm(self):
        self.move_distance(self.inv_y - self.inv_arm_y, self.cmd_s)
        self.Y_pos = self.Y_pos - (self.inv_y - self.inv_arm_y)
        self.current_pos = "arm"
        
    """
    Function to move the tiago towards the object.

    This function makes the tiago move towards the inventory table to pick up an object.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_arm_to_obj(self, x, y):
        self.obj_x = x
        self.obj_y = y + (self.inv_y - self.inv_arm_y) + 0.05
        if self.obj_x < 0:
            self.move_distance(abs(self.obj_x), self.cmd_a)
        else:
            self.move_distance(abs(self.obj_x), self.cmd_d)
        
        self.move_distance(self.obj_y, self.cmd_w)
        self.current_pos = "obj"
        
    """
    Function to move the tiago back from the inventory table.

    This function makes the tiago move back from the inventory table after grabbing an object so that it doesn't
    collide with any other object and can comfortably turn the base.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_obj_to_arm(self):
        self.move_distance(self.obj_y, self.cmd_s)
        if self.obj_x < 0:
            self.move_distance(abs(self.obj_x), self.cmd_d)
        else:
            self.move_distance(abs(self.obj_x), self.cmd_a)
        self.current_pos = "arm"

    """
    Function to move the tiago towards the target table

    This function makes the tiago move towards the target table, where it can place the picked up object.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_inv_to_tar(self):
        self.move_to_point(self.tar_x + self.half_base_width, self.home_y, -90)
        self.move_to_point(self.tar_x + self.half_base_width, self.tar_y + self.half_base_length, -110)
        self.current_pos = "tar"

    """
    Function to move the tiago from the target table to the home position.

    This function makes the tiago move from the target table to the home position. Here it asks the user for 
    the next command.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_tar_to_home(self):
        self.move_to_point(self.home_x, self.home_y, -95)
        self.current_pos = "home"

    """
    Function to move the tiago from the inventory table to the home position

    This function makes the tiago move from the target table to the home position. Here it asks the user for 
    the next command.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def move_from_inv_to_home(self):
        self.move_to_point(self.home_x, self.home_y, -95)
        self.current_pos = "home"

    """
    Function to move the tiago from the home position to the initial starting position.

    This function makes the tiago move from the home position to the initial starting position. This gets the robot 
    ready for shutdown process at the end of the task.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """
    def move_from_home_to_init(self):
        self.move_to_point(self.half_base_length, self.half_base_width, 100)
        self.current_pos = "init"

    """
    Function to move the tiago to a specified point and rotate by given amount.

    This function makes the tiago move to a given point and then rotate it by some orientation angle. 

        Args:
            self: passes the object calling the function as a parameter.
            x: x-coordinate of the target location
            y: y-coordinate of the target location
            ori: orientation angle by which to rotate at the target location
        Returns:
            no data is returned in this function.
    """

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

    """
    Function to move the tiago by a particular distance.

    This function makes the tiago move by a particular distance in the specified direction. 

        Args:
            self: passes the object calling the function as a parameter.
            distance: the distance by which to move the tiago.
            command: the direction in which to move the tiago.
        Returns:
            no data is returned in this function.
    """

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

    """
    Function to rotate the tiago by a particular angle.

    This function makes the tiago base rotate by a particular angle in the specified direction. 

        Args:
            self: passes the object calling the function as a parameter.
            angle: the angle by which to rotate the tiago.
            command: the direction in which to move the tiago.
        Returns:
            no data is returned in this function.
    """

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
    """
    Function to generate the twists for movement of the tiago.

    This function sets the required twist for the tiago base to move when a command is passed to it.

        Args:
            self: passes the object calling the function as a parameter.
            linear: the linear velocity of the twist
            anglular: the angular velocity of the twist 
        Returns:
            it returns the set twist that was defined by the linear and angular velocities
    """

    def generate_twist(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear[0]
        twist.linear.y = linear[1]
        twist.linear.z = linear[2]
        twist.angular.x = angular[0]
        twist.angular.y = angular[1]
        twist.angular.z = angular[2]

        return twist

    """
    Function to limit the angle range for movement of the base.

    This function sets the angle limits of the Tiago to a particular amount that should not be exceeded.
    It limits the angle range to 0 to 2*pi as all angles can be mapped onto those values.

        Args:
            self: passes the object calling the function as a parameter.
            angle: the specified angle which needs to be limited.
        Returns:
            it returns the new angle that the tiago has to follow within range.
    """

    def limit_angle_to_range(self, angle):
        if angle > math.pi:
            angle = angle - 2*math.pi
        if angle < -math.pi:
            angle = angle + 2*math.pi
        return angle
