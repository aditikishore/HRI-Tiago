# HRI-Tiago

Document link: https://livejohnshopkins-my.sharepoint.com/:w:/r/personal/vkolal1_jh_edu/_layouts/15/Doc.aspx?sourcedoc=%7B74F32930-1197-4B72-9275-2C8C4B4B47CA%7D&file=Document.docx&action=default&mobileredirect=true



Program Architecture:

All the files uploaded work in conjunction to get the TIAGo to move. To simplify the process, we use the main.py file that combines all the files and uses them in conjunction to make the TIAGo perform the physical HRI task.

The main.py file controls when and how each file is run and calls the other methods and functions in order to get the TIAGo to do the required task. 

move_body.py contains all the classes and methods for controlling the head, torso, arms and gripper of the TIAGo. It has various arbitrarily decided positions and controls that work to move the body parts of the TIAGo.

move_base.py contains the classes and methods for controlling the base of the robot. We have set some values by experimental conditions and have defined them as our intermediate setpoints for various motions that the TIAGo performs.

tiago_communication.py contains the classes and methods to help the TIAGo talk and listen to the user. It uses the Google Speech Recognition for this part.

marker_manager.py contains the classes and methods for detecting the Aruco markers on the objects using the camera of the TIAGo and sending out coordinates for the detected markers to the TIAGo for it to understand where the item for pick-up is.
