#!/usr/bin/env/python

import time
import rospy
import move_base
import move_body
import tiago_communication
import marker_manager


def init_ros():

    rospy.init_node('hri_node', anonymous=False)

def HRI_script():

    base = move_base.robot_base()
    print('Initialised base')
    body = move_body.robot_body()
    print('Initialised body')
    # talker = tiago_communication.Talker()
    print('Initialised talker')
    # listener = tiago_communication.Listener()
    print('Initialised listener')

    # test_motions(base, body)
    test_pickup(base, body)

    # base.move_from_init_to_home()

    # talker.talk('Hi, I am TIAGo, your personal helper robot. I can retrieve things for you from that table behind me.', block=True)
    # talker.talk('The things kept on the table are: a water bottle, pill bottle, coffee, jar of mixed nuts, and dried fruits.', block=True)
    # talker.talk('What would you like me to bring for you?.', block=True)

    # assist_loop(base, body, talker, listener)

    # base.move_from_home_to_init()

def assist_loop(base, body, talker, listener):

    exit = False

    while(not exit):

        item = listener.listen()

        if item == "water":
            print("water")
            talker.talk('I will bring you the water bottle', block=True)
        elif item == "pill":
            print("pill")
            talker.talk('I will bring you the pill bottle', block=True)
        elif item == "nuts":
            print("nuts")
            talker.talk('I will bring you the mixed nuts jar', block=True)
        elif item == "fruits":
            print("fruits")
            talker.talk('I will bring you the dried fruits jar', block=True)
        elif item == "coffee":
            print("coffee")
            talker.talk('I will bring you your coffee', block=True)
        elif item == "thank":
            print("exit")
            talker.talk('The pleasure was all mine. I will now go to my starting position for shutdown.', block=True)
            exit = True
            break
        else:
            print("I did not catch any keywords")
            talker.talk('I was not able to understand you, could you please repeat your request?', block=True)
            continue


        base.move_from_home_to_inv()
        time.sleep(5)
        body.raise_torso()
        time.sleep(5)
        body.look_at_inv()
        time.sleep(2)
        prompt = 'I am trying to pick up ' + item + ' from the table but it looks like my arms are not working'
        talker.talk(prompt, block=True)
        body.center_torso()
        time.sleep(5)
        body.look_straight()
        time.sleep(2)

        base.move_from_inv_to_tar()
        time.sleep(5)
        talker.talk('Unfortunately, I was unable to retrieve your item', block=True)

        base.move_from_tar_to_home()
        time.sleep(5)
        talker.talk('Could I get you something else?', block=True)

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

    # base.move_distance(0.05, base.cmd_w)

    base.move_from_init_to_home()
    time.sleep(2)

    base.move_from_home_to_inv()
    time.sleep(2)

    body.raise_torso()
    time.sleep(5)

    body.head_mgr('disable')
    time.sleep(2)
    
    body.look_at_inv()
    time.sleep(2)

    marker.get_markers()

    body.extend_right_arm()
    time.sleep(5)

    marker.calc_arm_to_obj()

    # body.center_torso()
    # time.sleep(5)
    # body.look_straight()
    # time.sleep(2)
    # base.move_from_inv_to_tar()
    # time.sleep(2)
    # base.move_from_tar_to_home()
    # time.sleep(2)
    # base.move_from_home_to_init()
    


if __name__ == '__main__':
    init_ros()

    try:
        HRI_script()
    except KeyboardInterrupt:
        print("You typed CTRL + C")
    else:
        print("Script executed successfully")
