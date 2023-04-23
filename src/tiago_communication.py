#!/usr/bin/env python

from actionlib import SimpleActionClient 
from pal_interaction_msgs.msg import TtsAction, TtsGoal
import rospy
from std_msgs.msg import String
import sys
import time
import speech_recognition as sr
import spacy


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
        
        # A mapping from keywords or phrases to commands
        self.keywords_to_command = {'water': ['water'],
                                    'pill': ['pill'],
                                    'coffee': ['coffee'], 
                                    'fruits': ['fruits'],
                                    'nuts': ['nuts']}
        

if __name__ == '__main__':
    
    # Initialize this as a ROS node
    rospy.init_node('speech_node', anonymous=True)
    
    # Create a talker object
    talker = Talker()
    talker.talk('hello friends my name is TIAGo', language='en_GB', block=True)

    # Create recognizer object
    r = sr.Recognizer()
    
    nlp = spacy.load("en_core_web_sm")
    
    # Use default microphone as audio source 
    with sr.Microphone() as source:
        # adjust for noise 
        r.adjust_for_ambient_noise(source)
        # prompt user to say something
        print("please give me a command")
        talker.talk('please give me a command', language='en_GB', block=True)
        # sleep for 2 seconds 
        time.sleep(2)

        # listen for user input 
        t_end = time.time() + 5
        while time.time() < t_end:
            print("listening")
            audio = r.listen(source)
        
        print("got it")
    try:
        # recognize speech using Google Speech Recognition 
        text = r.recognize_google(audio)

        command = nlp(text)
        talker.talk(text, language='en_GB', block=True)
        for token in command:
            print(token, token.idx)

            if token.text == "water":
                print("water")
                talker.talk('I will get you water', language='en_GB', block=True)
            elif token.text == "pill":
                print("pill")
                talker.talk('I will get you the pill bottle', language='en_GB', block=True)
            elif token.text == "fruit":
                print("fruit")
                talker.talk('I will get you the fruit container', language='en_GB', block=True)
            elif token.text == "nut":
                print("nut")
                talker.talk('I will get you the box of nuts', language='en_GB', block=True)
            elif token.text == "coffee":
                print("coffee")
                talker.talk('I will grab your coffee', language='en_GB', block=True)
            else:
                print("Oops")
    except sr.UnknownValueError:
        print("Oops! Unable to understand the audio input.")
    
    except sr.RequestError as e:
        print("Oops! Could not request results from Google Speech Recognition service; {0}".format(e))