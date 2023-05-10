"""
Main - main ROSPY code for controlling all our files, nodes, and codes. 

In this file, we created our base algorithm for the entire task (workflow) that was
required to do. We import all our files into this file and run this main file to get
the TiaGo to do the work we expect it to do. 

This code is required along with the other codes we have written to call them in the
correct order for our entire task to execute properly.

Usage:
    1. Import the necessary modules:
        import time
        import rospy
        import move_base
        import move_body
        import tiago_communication
        import marker_manager
        
    2. Call the various functions of other classes (examples):
        base.move_from_home_to_init()
        body.open_gripper_right()
        markers = marker.marker_dict

    3. Run the node
        if __name__ == '__main__':
            init_ros()
"""
#!/usr/bin/env/python

import time
import rospy
import move_base
import move_body
import tiago_communication
import marker_manager

"""
Function to initialise the main ros node.

This function initialised the ros node needed for all the communication between the rospy
code and the ROS system.

"""

def init_ros():

    rospy.init_node('hri_node', anonymous=False)

"""
Main function to run all the tasks required for this demo.

This is the main function that will call all nodes and processes in order to
perform our physical HRI task. It will divide the work and call the appropriate methods
as and when needed and tell the TiaGo exactly what is to be done.

"""
def HRI_script():

    #initialise class objects
    base = move_base.robot_base()
    body = move_body.robot_body()
    talker = tiago_communication.Talker()
    listener = tiago_communication.Listener()
    marker = marker_manager.marker_manager()
    
    # test_motions(base, body)

    base.move_from_init_to_home()

    talker.talk('Hi, I am TIAGo, your personal helper robot. I can retrieve things for you from that table behind me.', block=True)
    talker.talk('The things kept on the table are: a pill bottle, water bottle, oats, jar of mixed nuts, and multi-vitamins.', block=True)
    talker.talk('What would you like me to bring for you?.', block=True)

    #run a loop to fetch objects based on voice commands
    assist_loop(base, body, talker, listener, marker)
    # test_assist_loop(base, body, listener, marker)

    #park the robot when the user does not want more items and the loop exits
    print ('Moving to init for shutdown')
    base.move_from_home_to_init()

"""
Function to keep running the main HRI project tasks on loop.

This function keeps running the HRI project Code on loop. The loop it follows is ask the TiaGo for a command to pick up an item,
go to the table to pick the item up, place the item on the target table, and then ask the user for the next command. 

    Args:
        base: object of the robot_base class used for moving the base of the TiaGo
        body: object of the robot_body class used for moving the head, torso and arms of the TiaGo
        talker: object of the Talker class used for telling the TiaGo to speak
        listener: object of the Listener class used for telling the TiaGo to record the commands the user gives
        marker: object of the marker_manager class that is used for tracking the Aruco markers
    Returns:
        no data is returned in this function.
"""
def assist_loop(base, body, talker, listener, marker):

    exit = False

    while(not exit):

        #get command
        item = listener.listen()
        print (item)
        
        #if no keywords detected, go back to start of loop and ask for command again
        if item in ["nothing", "unknown"]:
            print("I did not catch any keywords")
            talker.talk('I was not able to understand you, could you please repeat your request?', block=True)
            continue
        
        #if command is thank you, terminate the loop and exit the function
        if item in ["thank", "thanks"]:
            print("exit")
            talker.talk('The pleasure was all mine. I will now go to my starting position for shutdown.', block=True)
            exit = True
            break
        
        #acknowledge and confirm the requested item
        talker.talk("I will bring you your " + listener.keyword_to_id.get(item)[1], block=True)

        #go to inventory table
        base.move_from_home_to_inv()
        body.head_mgr('disable')
        time.sleep(5)
        
        body.raise_torso()
        time.sleep(5)
        
        body.look_down()
        time.sleep(2)
        
        #start marker detection node and get dict of markers and their posiitons
        talker.talk('Please help me see.', block=True)
        markers = marker.get_markers()

        #check if requested object's marker ID is in dict. If not, move the head to a better position and attempt detection again.
        if listener.keyword_to_id.get(item)[0] not in markers:
            print("Not able to see ", item)
            talker.talk("I cannot see the " + listener.keyword_to_id.get(item)[1], block=True)
            talker.talk('I will try looking again', block=False)
            body.look_down_more()
            time.sleep(2)
            talker.talk('Please help me see.', block=True)
            markers = marker.get_markers()
       
        #check again if requested object's marker ID is in dict.
        if listener.keyword_to_id.get(item)[0] in markers:
            body.head_mgr('enable')
            print('Found requested object marker')
            
            #move to a position where arm can extend
            base.move_from_inv_to_arm()
            time.sleep(2)
        
            #acknowledge detection of object
            talker.talk("I see the " + listener.keyword_to_id.get(item)[1], block=True)
            talker.talk('I will pick it up with my arm now.', block=False)

            body.extend_right_arm()
            time.sleep(5)
            
            #calculate the X and Y distance from gripper to object
            X_off, Y_off = marker.calc_arm_to_obj(listener.keyword_to_id.get(item)[0])
            print('X offset: ', X_off)
            print('Y offset: ', Y_off)

            #move to grasp object
            base.move_from_arm_to_obj(-Y_off, X_off)
            time.sleep(5)
            
            body.close_gripper_right()
            time.sleep(2)

            #move back to a safe position before turning towards target table
            base.move_from_obj_to_arm()
            time.sleep(2)
            
            talker.talk('I am moving to the target table now to drop off the ' + listener.keyword_to_id.get(item)[1] + '. Please stand clear.', block=False)
            base.move_from_inv_to_tar()
            time.sleep(2)
            
            body.lower_torso()
            time.sleep(3)

            #drop off the item
            talker.talk('Here you go, your ' + listener.keyword_to_id.get(item)[1] + ' is on the target table', block=False)
            body.open_gripper_right()
            time.sleep(2)
            
            body.retract_right_arm()
            time.sleep(2)
            
            #move to home and ready for further commands
            base.move_from_tar_to_home()
            time.sleep(5)
        
        else:
            #if the requested marker ID was not found two times in a row, move back to home and accept another command.
            body.head_mgr('enable')
            print('Still not able to see marker')
            talker.talk("Sorry, I was not able to find the " + listener.keyword_to_id.get(item)[1], block=True)
            body.center_torso()
            time.sleep(5)
            base.move_from_inv_to_home()
            time.sleep(5)
        
        #ask for another command before looping
        talker.talk('Could I get you something else?', block=True)

"""
Function to test that individual objects of the code are working well in sync.

This is a function used to test the previous section of code. It does not have any talking commands and just performs the task as is.
It has some hard-coded values for testing whether the code works as we intend it to.

    Args:
        base: object of the robot_base class used for moving the base of the TiaGo
        body: object of the robot_body class used for moving the head, torso and arms of the TiaGo
        listener: object of the Listener class used for telling the TiaGo to record the commands the user gives
        marker: object of the marker_manager class that is used for tracking the Aruco markers
    Returns:
        no data is returned in this function.
"""
def test_assist_loop(base, body, listener, marker):

    exit = False

    while(not exit):

        item = 'vitamins'
        item = listener.listen()
        print (item)
        
        if item in ["nothing", "unknown"]:
            print("I did not catch any keywords")
            continue
        
        if item in ["thank", "thanks"]:
            print("exit")
            exit = True
            break
        
        #go to inv table
        base.move_from_home_to_inv()
        body.head_mgr('disable')
        time.sleep(5)
        
        body.raise_torso()
        time.sleep(5)
        
        body.look_down()
        time.sleep(2)
        
        # markers = marker.get_markers()
        markers = marker.marker_dict

        if listener.keyword_to_id.get(item)[0] not in markers:
            print("Not able to see ", item)
            body.look_down_more()
            time.sleep(2)
            markers = marker.marker_dict
            # markers = marker.get_markers()
       
        if listener.keyword_to_id.get(item)[0] in markers:
            body.head_mgr('enable')
            print('Found requested object marker')
            
            base.move_from_inv_to_arm()
            time.sleep(2)
        
            body.extend_right_arm()
            time.sleep(5)
            
            X_off, Y_off = marker.calc_arm_to_obj(listener.keyword_to_id.get(item)[0])
            # X_off = 0.4
            # Y_off = 0.1
            print('X offset: ', X_off)
            print('Y offset: ', Y_off)
            base.move_from_arm_to_obj(-Y_off, X_off)
            time.sleep(5)
            
            body.close_gripper_right()
            time.sleep(2)
            base.move_from_obj_to_arm()
            time.sleep(2)
            
            base.move_from_inv_to_tar()
            time.sleep(2)
            
            body.lower_torso()
            time.sleep(3)
            body.open_gripper_right()
            time.sleep(2)
            
            body.retract_right_arm()
            time.sleep(2)
            
            base.move_from_tar_to_home()
            time.sleep(5)
            
        else:
            body.head_mgr('enable')
            body.center_torso()
            time.sleep(5)
            print('Still not able to see marker')
            base.move_from_inv_to_home()
            time.sleep(5)


"""
Function to test the base movement for deciding some exact locations for the base such as home and intermediate home points.

This function is used to move the base of the robot for testing whether we can find out exact locations where we want the base
to move without colliding anywhere in the workspace.

    Args:
        base: object of the robot_base class used for moving the base of the TiaGo
        body: object of the robot_body class used for moving the head, torso and arms of the TiaGo
    Returns:
        no data is returned in this function.
"""
def test_motions(base, body):

    base.move_from_init_to_home()
    time.sleep(2)
    base.move_from_home_to_inv()
    time.sleep(2)
    body.raise_torso()
    time.sleep(5)
    body.look_at_inv()
    time.sleep(2)
    body.center_torso()
    time.sleep(5)
    body.look_straight()
    time.sleep(2)
    base.move_from_inv_to_tar()
    time.sleep(2)
    base.move_from_tar_to_home()
    time.sleep(2)
    base.move_from_home_to_init()

if __name__ == '__main__':
    init_ros()

    try:
        HRI_script()
    except KeyboardInterrupt:
        print("You typed CTRL + C")
    else:
        print("Script executed successfully")
