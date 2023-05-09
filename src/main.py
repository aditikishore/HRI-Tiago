#!/usr/bin/env/python

import time
import rospy
import move_base
import move_body
import tiago_communication
import marker_manager

#change head controller to jt
#be closer to table when looking down. arm was at edge of table from current pos
#target table was almost 20deg

def init_ros():

    rospy.init_node('hri_node', anonymous=False)

def HRI_script():

    base = move_base.robot_base()
    
    body = move_body.robot_body()
    
    # talker = tiago_communication.Talker()
    
    listener = tiago_communication.Listener()
    
    marker = marker_manager.marker_manager()
    

    # test_motions(base, body)
    # test_pickup(base, body)

    base.move_from_init_to_home()
    
    test_assist_loop(base, body, listener, marker)

    # talker.talk('Hi, I am TIAGo, your personal helper robot. I can retrieve things for you from that table behind me.', block=True)
    # talker.talk('The things kept on the table are: a pill bottle, water bottle, oats, jar of mixed nuts, and multi-vitamins.', block=True)
    # talker.talk('What would you like me to bring for you?.', block=True)

    # assist_loop(base, body, talker, listener, marker)

    base.move_from_home_to_init()
    print ('moving to init')

def assist_loop(base, body, talker, listener, marker):

    exit = False

    while(not exit):

        item = listener.listen()
        print (item)
        
        if item in ["nothing", "unknown"]:
            print("I did not catch any keywords")
            talker.talk('I was not able to understand you, could you please repeat your request?', block=True)
            continue
        
        if item in ["thank", "thanks"]:
            print("exit")
            talker.talk('The pleasure was all mine. I will now go to my starting position for shutdown.', block=True)
            exit = True
            break
        
        talker.talk("I will bring you your " + listener.keyword_to_id.get(item)[1], block=True)

        #go to inv table
        base.move_from_home_to_inv()
        body.head_mgr('disable')
        time.sleep(5)
        
        body.raise_torso()
        time.sleep(5)
        
        body.look_down()
        time.sleep(2)
        
        markers = marker.get_markers()

        if listener.keyword_to_id.get(item)[0] not in markers:
            print("Not able to see ", item)
            talker.talk("I cannot see the " + listener.keyword_to_id.get(item)[1], block=True)
            talker.talk('I will try looking again', block=False)
            body.look_down_more()
            time.sleep(2)
            markers = marker.get_markers()
       
        if listener.keyword_to_id.get(item)[0] in markers:
            body.head_mgr('enable')
            print('Found requested object marker')
            
            base.move_from_inv_to_arm()
            time.sleep(2)
        
            talker.talk("I see the " + listener.keyword_to_id.get(item)[1], block=True)
            talker.talk('I will pick it up with my arm now.', block=False)

            body.extend_right_arm()
            time.sleep(5)
            
            X_off, Y_off = marker.calc_arm_to_obj(listener.keyword_to_id.get(item)[0])
            print('X offset: ', X_off)
            print('Y offset: ', Y_off)
            base.move_from_arm_to_obj(-Y_off, X_off)
            time.sleep(5)
            
            body.close_gripper_right()
            time.sleep(2)
            base.move_from_obj_to_arm()
            time.sleep(2)
            
            talker.talk('I am moving to the target table now to drop off the '+ listener.keyword_to_id.get(item)[1], '. Please stand clear.', block=False)
            base.move_from_inv_to_tar()
            time.sleep(2)
            
            body.lower_torso()
            time.sleep(3)
            talker.talk('Here you go, your '+ listener.keyword_to_id.get(item)[1], ' is on the target table', block=False)
            body.open_gripper_right()
            time.sleep(2)
            
            body.retract_right_arm()
            time.sleep(2)
            
            base.move_from_tar_to_home()
            time.sleep(5)
        
        else:
            body.head_mgr('enable')
            print('Still not able to see marker')
            talker.talk("Sorry, I was not able to find the " + listener.keyword_to_id.get(item)[1], block=True)
            body.center_torso()
            time.sleep(5)
            base.move_from_inv_to_home()
            time.sleep(5)
        
        talker.talk('Could I get you something else?', block=True)
            
def test_assist_loop(base, body, listener, marker):

    exit = False

    while(not exit):

        item = 'vitamins'
        # item = listener.listen()
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
        
        markers = marker.get_markers()
        # markers = marker.marker_dict

        if listener.keyword_to_id.get(item)[0] not in markers:
            print("Not able to see ", item)
            body.look_down_more()
            time.sleep(2)
            # markers = marker.marker_dict
            markers = marker.get_markers()
       
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

def test_pickup(base, body):

    marker = marker_manager.marker_manager()
    listner = tiago_communication.Listener()

    base.move_distance(0.05, base.cmd_w)
    
    
    
    marker.get_arm_transform()
    
    body.extend_right_arm()
    
    marker.get_arm_transform()
    
    body.retract_right_arm()
    
    body.look_down()

    base.move_from_init_to_home()
    time.sleep(2)

    ## Listen for command
    item = listner.listen()
    if item in ["nothing","unknown"]:
        #handle nothing case crappy listen
        print(item)
        return 
    if item in ["thank"]:
        
        pass
    # base.move_from_home_to_inv()
    time.sleep(2)

    body.raise_torso()
    time.sleep(5)
    #  From the above lines we will be in front of the table, Close enought that we can see but cant extend
    body.head_mgr('disable')
    time.sleep(5)
    print('should be looking at table now')
    markers = marker.get_markers()
    body.head_mgr('enable')

    print('got markers')
    ## Given the markers and the item, find the marker that corresponds to the item
    if listner.keyword_to_id.get(item)[0] in markers:
        print('found marker')
        body.move_from_inv_to_arm()
        time.sleep(5)
        
        body.extend_right_arm()
        time.sleep(5)
        # Look at arm offset
        xoff, yoff = marker.calc_arm_to_obj(listner.keyword_to_id.get(item)[0])
        body.move_from_arm_to_obj(yoff, -xoff)
        time.sleep(5)
        ## Move to the table and pick up the item
        
    else:
        ## ITem marker not found
        print('did not find marker')
        talkerResponse = 'I was not able to understand you, could you please repeat your request?'
        


if __name__ == '__main__':
    init_ros()

    try:
        HRI_script()
    except KeyboardInterrupt:
        print("You typed CTRL + C")
    else:
        print("Script executed successfully")
