#!/usr/bin/env python

from actionlib import SimpleActionClient 
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospy

def init_ros():
    rospy.init_node('speech_node', anonymous=False)

class Talker(object):
    def __init__(self):
        # Connect to text-to-speech action server
        self.ac = SimpleActionClient('/tts', TtsAction)
        self.ac.wait_for_server()

    def talk(self, text, language='en_GB', block=True):
        goal = TtsGoal()
        goal.rawtext.lang_id = language
        goal.rawtext.text = text

        if block:
            self.ac.send_goal_and_wait(goal)
        else:
            self.ac.send_goal(goal)

if __name__ == '__main__':
    #init_ros()
    # Initialize this as a ROS node
    rospy.init_node('speech_node', anonymous=True)
    # Create a talker object
    talker = Talker()
    talker.talk('hello friends', language='en_GB', block=True)
