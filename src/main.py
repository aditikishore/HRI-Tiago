#!/usr/bin/env/python

import sys
import time
import rospy
from pal_interaction_msgs.msg import TtsAction, TtsGoal
from std_msgs.msg import String
import speech_recognition as sr
import spacy
import move_base
from control_msgs.msg import PointHeadAction
from control_msgs.msg import PointHeadGoal
import actionlib
from actionlib import SimpleActionClient 
from geometry_msgs.msg import PointStamped


def init_ros():
    
    rospy.init_node('hri_node', anonymous=False)
    
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
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'water': ['water'],
                                    'pill': ['pill'],
                                    'coffee': ['coffee'], 
                                    'fruits': ['fruits'],
                                    'nuts': ['nuts']}
    
    def listen():
        # Use default microphone as audio source 
        with sr.Microphone() as source:
            # adjust for noise 
            r.adjust_for_ambient_noise(source)
            # prompt user to say something
            print("please give me a command")
            talker.talk('May I get you anything', language='en_GB', block=True)
            # sleep for half a second 
            time.sleep(0.5)
            # listen for user input 
            t_end = time.time() + 5
            while time.time() < t_end:
                print("listening")
                audio = r.listen(source)
        
            print("got it")
        try:
            # recognize speech using Google Speech Recognition 
            text = r.recognize_google(audio)
            Listener.keywordCheck(text, audio)
        except sr.UnknownValueError:
            print("Oops! Unable to understand the audio input.")
        except sr.RequestError as e:
            print("Oops! Could not request results from Google Speech Recognition service; {0}".format(e))

    def keywordCheck(text, audio):
        text = r.recognize_google(audio)
        command = nlp(text)
        for token in command:
            print(token, token.idx)

            if token.text == "water":
                print("water")
                talker.talk('I will bring you the water bottle', language='en_GB', block=True)
            elif token.text == "pill":
                print("pill")
                talker.talk('I will bring you the pill bottle', language='en_GB', block=True)
            elif token.text == "nuts":
                print("nuts")
                talker.talk('I will bring you the mixed nuts jar', language='en_GB', block=True)
            elif token.text == "fruits":
                print("fruits")
                talker.talk('I will bring you the dried fruits jar', language='en_GB', block=True)
            elif token.text == "coffee":
                print("coffee")
                talker.talk('I will bring you your coffee', language='en_GB', block=True)
            else:
                print("I did not catch any keywords")

if __name__=='__main__':
    init_ros()

    base = move_base.robot_base()
    base.move_from_init_to_home()

    # Create a talker object and introduce yourself 
    talker = Talker()
    talker.talk('Hello friends my name is TIAGo. I can retrieve objects for you through verbal requests. Some examples include water bottles, pill bottles, coffee, mixed nuts, and dried fruits.', language='en_GB', block=True)

    # Create recognizer object
    r = sr.Recognizer()
    nlp = spacy.load("en_core_web_sm")
    Listener.listen()

    base.move_from_home_to_inv()
    time.sleep(5)
    talker.talk('I am placing your item on the target table now', language='en_GB', block=True)

    base.move_from_inv_to_tar()
    time.sleep(5)
    talker.talk('I have delivered your item to the target table', language='en_GB', block=True)

    base.move_from_tar_to_home()
    time.sleep(5)
    talker.talk('Now that I have delivered the item you requested, I can get you something else in a moment', language='en_GB', block=True)
    Listener.listen()

    # camera_frame = "/xtion_rgb_optical_frame"
    # base_frame = "base_link"

    # point = PointStamped()
    # point.header.frame_id = base_frame
    # point.header.stamp = rospy.Time.now()
    # point.point.x = 2       #distance outward
    # point.point.y = 0       #distance left
    # point.point.z = 0.3     #distance up

    # goal = PointHeadGoal()
    # goal.pointing_frame = camera_frame
    # goal.pointing_axis.x = 0.0
    # goal.pointing_axis.y = 0.0
    # goal.pointing_axis.z = 1.0
    # goal.min_duration = rospy.Duration(1.0)
    # goal.max_velocity = 0.25
    # goal.target = point

    # client = actionlib.SimpleActionClient('/head_controller/point_head_action', PointHeadAction)
    # client.wait_for_server()

    # client.send_goal(goal)
    # client.wait_for_result()