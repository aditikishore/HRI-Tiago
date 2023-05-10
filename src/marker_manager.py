"""
Marker_Manager - ROSPY code for starting the Aruco Node and taking in the pose of the markers 
on the table.

In this file, we start the Aruco Node and take in marker pose messages to get the coordinates (x,
y, z) of the marker put up on the table to determine the exact position that the Tiago Arm has to move to.
We take in marker values from the aruco_msgs.msgs node and perform the required transformation to get the 
final position that the arm has to move to. 

This code is used in the main file, wherein we used marker_manager() as the calling function from the 
marker_manager file, and use that to tell the Tiago where to move it's arm.

Usage:
    1. Import the necessary modules:
        import rospy
        import time

        from geometry_msgs.msg import Pose, PoseStamped
        from aruco_msgs.msg import Marker, MarkerArray

        import tf
    
    2. Create an instance of the marker_manager class (in main.py):
        marker = marker_manager.marker_manager()
        
    3. Call the various functions of the class using the created instance in main.py:
        marker.get_markers()
        marker.calc_arm_to_obj()

    4. Run the node using the main() function in main.py:
        if __name__ == '__main__':
            HRI_Script()

        (in HRI_Script():)
            test_pickup(base, body)
"""

#!/usr/bin/env/python

import rospy
import time

from geometry_msgs.msg import Pose, PoseStamped
from aruco_msgs.msg import Marker, MarkerArray
import subprocess
import tf

    

class marker_manager:

    """
    Function to initialise the marker_array used to record marker pose values.
    
    Define the marker_array list that we will use and define an object of the array.
    Also defines a listener node that collects data from the Tiago.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def __init__(self):

        #dictionary to store marker IDs and positions initialised with hard coded values for testing in gazebo
        self.marker_dict = dict()
        self.marker_dict[9] = {'x': 1.0, 'y': -0.6, 'z': 1}
        self.marker_dict[10] = {'x': 0.8, 'y': -0.3, 'z': 1}
        self.marker_dict[11] = {'x': 0.8, 'y': 0.0, 'z': 1}
        self.marker_dict[8] = {'x': 0.8, 'y': 0.3, 'z': 1}
        self.marker_dict[12] = {'x': 1.0, 'y': 0.6, 'z': 1}


        self.msg = MarkerArray()
        self.tf_listener = tf.TransformListener()
        print('Initialised marker manager')

    """
    Function to start the Aruco Node and take in the message from the node.
    
    This function waits till we get a message (information) from the Aruco Node and then 
    saves that as a message to be used later for computation.
    
        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
            a confirmation message is printed on the terminal at the end of this function.
    """

    def get_markers(self):
        self.marker_dict = dict()
        print("Please start ARUCO node")
        # subprocess.run(["rosrun", "aruco_ros", "marker_publish_node"])
        time.sleep(2)
        msg = rospy.wait_for_message("/aruco_marker_publisher/markers", MarkerArray, timeout=None)
        self.store_markers(msg)
        self.msg = msg
        # subprocess.run(["rosnode", "kill", "/aruco_marker_publisher"])
        print("Message received, please shutdown node within 5 sec")
        time.sleep(5)
        return self.marker_dict

    """
    Function to record the marker coordinates of the Aruco Node.
    
    This function stores the marker coordinates received from the Aruco node as x, y, z coordinate values along with
    the marker ID of the message.
    
        Args:
            self: passes the object calling the function as a parameter.
            msg: the message which contains data of the ID, x, y, z values of the marker node.
        Returns:
            no data is returned in this function.
    """

    def store_markers(self, msg):
        for marker in msg.markers:
            self.marker_dict[marker.id] = {'x': marker.pose.pose.position.x, 'y': marker.pose.pose.position.y, 'z': marker.pose.pose.position.z}
            print("Got marker ID: ", marker.id)

        # print("Stored marker List: ")
        # print(self.marker_dict)

    """
    Function to record the arm position. 
    
    This function calls get_arm_transform() from this file which gives it the x, y and z coordinate
    of the gripper position. 
    
        Args:
            self: passes the object calling the function as a parameter.
            id: 
        Returns:
            no data is returned in this function.
    """

    def calc_arm_to_obj(self, id):
        [x, y, z] = self.get_arm_transform()
        # print("arm coords ")
        # print("X: ", x)
        # print("Y: ", y)
        # print("Z: ", z)
        X_off  = self.marker_dict[id]['x'] - x
        Y_off = self.marker_dict[id]['y'] - y
        
        return X_off, Y_off

    """
    Function to get the position of the arm.

    When this function gets called, it takes in the position and orientation of the arm using
    PoseStamped() function. It then waits and records the transform coordinates of the arm.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            the x, y and z coordinates of the object frame that calls this function.
    """

    def get_arm_transform(self):
        p1 = PoseStamped()
        p1.header.frame_id = "gripper_right_grasping_frame"
        p1.pose.orientation.w = 1.0    # Neutral orientation
        p_in_base = self.tf_listener.transformPose("/base_footprint", p1)
        print (p_in_base)
        return p_in_base.pose.position.x, p_in_base.pose.position.y, p_in_base.pose.position.z
        

if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('marker_man', anonymous=True)


    marker = marker_manager()
    marker.get_markers()