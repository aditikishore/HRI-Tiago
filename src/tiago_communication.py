#!/usr/bin/env python

from actionlib import SimpleActionClient 
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospy
from std_msgs.msg import String
import sys

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

class Listener(object):
    def __init__(self):

        # Subscribe to the /ann_recognizer/output topic to receive voice commands.
        rospy.Subscriber("chatter", String, self.speech_callback)
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'water': ['water'],
                                    'pill': ['pill'],
                                    'coffee': ['coffee'], 
                                    'fruits': ['fruits'],
                                    'nuts': ['nuts']}

    def get_command(self, data):
        # Attempt to match the recognized word or phrase to the 
        # keywords_to_command dictionary and return the appropriate
        # command
        for (command, keywords) in self.keywords_to_command.iteritems():
            for word in keywords:
                if data.find(word) > -1:
                    # Wait time for other command, in seconds
                    self.time1 = rospy.get_rostime()
                    self.wait_time = 10
                    return command
    def speech_callback(self, msg):
        if self.TIAGo:
            self.time2 = rospy.get_rostime()
            if self.time2.secs < self.time1.secs+self.wait_time:
                command = self.get_command(msg.data)
            else:
                self.TIAGo = False
                self.wait_time = self.wait_time_initial
                return
        else:
            return
        
        if command == 'water':
            talker.talk('water', language='en_GB', block=True)
        elif command == 'pill':
            talker.talk('pill', language='en_GB', block=True)
        elif command == 'coffee':
            talker.talk('coffee', language='en_GB', block=True)
        elif command == 'nuts':
            talker.talk('nuts', language='en_GB', block=True)
        elif command == 'fruits':
            talker.talk('fruits', language='en_GB', block=True)
        else:
            return

if __name__ == '__main__':
    #init_ros()
    # Initialize this as a ROS node
    rospy.init_node('speech_node', anonymous=True)
    # Create a talker object
    talker = Talker()
    talker.talk('hello friends', language='en_GB', block=True)
