# HRI-Tiago

Detailed Instructions:
This codebase contains code and files that are needed for the Physical Human-Robot Interaction Task. The nodes used in this file is in alignment with the TIAGo Tutorials for ROS. The code developed here was modified to suit the task that was needed and has been written in such a way as to make it easy to understand and replicate if needed. 
The download procedure is the same as defined in the ROS Tutorials.
We added a few extra modules such as a Google Speech Recognition for talking and listening to the TIAGo.


Program Architecture:

All the files uploaded work in conjunction to get the TIAGo to move. To simplify the process, we use the main.py file that combines all the files and uses them in conjunction to make the TIAGo perform the physical HRI task.

The main.py file controls when and how each file is run and calls the other methods and functions in order to get the TIAGo to do the required task. 

move_body.py contains all the classes and methods for controlling the head, torso, arms and gripper of the TIAGo. It has various arbitrarily decided positions and controls that work to move the body parts of the TIAGo.

move_base.py contains the classes and methods for controlling the base of the robot. We have set some values by experimental conditions and have defined them as our intermediate setpoints for various motions that the TIAGo performs.

tiago_communication.py contains the classes and methods to help the TIAGo talk and listen to the user. It uses the Google Speech Recognition for this part.

marker_manager.py contains the classes and methods for detecting the Aruco markers on the objects using the camera of the TIAGo and sending out coordinates for the detected markers to the TIAGo for it to understand where the item for pick-up is.
