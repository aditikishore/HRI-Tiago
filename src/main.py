#!/usr/bin/env/python

import time
import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String
import speech_recognition as sr
import spacy
import move_base
import move_body
import tiago_communication

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def init_ros():

    rospy.init_node('hri_node', anonymous=False)

def HRI_script():
    base = move_base.robot_base()
    body = move_body.robot_body()
    talker = tiago_communication.Talker()
    # listener = tiago_communication.Listener()

    base.move_distance(1.25 + base.half_base_length, base.cmd_w)

    
    # body.lower_torso()
    # time.sleep(5)

    # base.move_from_init_to_home()
    # time.sleep(2)
    # base.move_from_home_to_inv()
    # time.sleep(2)
    body.raise_torso()
    time.sleep(5)
    body.look_at_inv()
    time.sleep(2)
    body.center_torso()
    time.sleep(5)
    body.look_straight()
    time.sleep(2)
    # base.move_from_inv_to_tar()
    # time.sleep(2)
    # base.move_from_tar_to_home()

    # talker.talk('Hi, I am TIAGo, your personal helper robot. I can retrieve things for you from that table behind me.', block=True)
    # talker.talk('The things kept on the table are: a water bottle, pill bottle, coffee, jar of mixed nuts, and dried fruits.', block=True)
    # talker.talk('What would you like me to bring for you?.', block=True)

    # # Create recognizer object

    # item = listener.listen()

    # if item == "water":
    #     print("water")
    #     talker.talk('I will bring you the water bottle', block=True)
    # elif item == "pill":
    #     print("pill")
    #     talker.talk('I will bring you the pill bottle', block=True)
    # elif item == "nuts":
    #     print("nuts")
    #     talker.talk('I will bring you the mixed nuts jar', block=True)
    # elif item == "fruits":
    #     print("fruits")
    #     talker.talk('I will bring you the dried fruits jar', block=True)
    # elif item == "coffee":
    #     print("coffee")
    #     talker.talk('I will bring you your coffee', block=True)
    # else:
    #     print("I did not catch any keywords")

    # base.move_from_home_to_inv()
    # time.sleep(5)
    # talker.talk('I am placing your item on the target table now', block=True)

    # base.move_from_inv_to_tar()
    # time.sleep(5)
    # talker.talk('I have delivered your item to the target table', block=True)

    # base.move_from_tar_to_home()
    # time.sleep(5)
    # talker.talk('Now that I have delivered the item you requested, I can get you something else in a moment', block=True)
    # listener.listen()


if __name__ == '__main__':
    init_ros()

    try:
        HRI_script()
    except KeyboardInterrupt:
        print("You typed CTRL + C")
    else:
        print("Script executed successfully")
