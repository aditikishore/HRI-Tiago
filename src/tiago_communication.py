#!/usr/bin/env python

import rospy
import time
import speech_recognition as sr
import spacy

from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal


def init_ros():
    rospy.init_node('speech_node', anonymous=False)


class Talker:
    def __init__(self):
        self.language = 'en_GB'

        # Connect to text-to-speech action server
        self.ac = SimpleActionClient('/tts', TtsAction)
        print('waiting for tts server')
        self.ac.wait_for_server()

    def talk(self, text, block=True):
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = text

        if block:
            self.ac.send_goal_and_wait(goal)
        else:
            self.ac.send_goal(goal)


class Listener:
    def __init__(self):

        # A mapping from keywords or phrases to commands
        self.keywords_to_ID = {'water': ['water'],
                               'pill': ['pill'],
                               'coffee': ['coffee'],
                               'fruits': ['fruits'],
                               'nuts': ['nuts']}

        self.recognizer = sr.Recognizer()
        self.nlp = spacy.load("en_core_web_sm")

    def listen(self):
        # Use default microphone as audio source
        item = 'nothing'
        with sr.Microphone() as source:

            # adjust for noise
            self.recognizer.adjust_for_ambient_noise(source)

            # prompt user to say something
            print("Please give me a command")

            # sleep for half a second
            time.sleep(0.5)

            # listen for user input
            t_end = time.time() + 5
            while time.time() < t_end:
                print("listening")
                audio = self.recognizer.listen(source)

            print("Heard something")

        try:
            # recognize speech using Google Speech Recognition
            item = self.keywordCheck(audio)
        except sr.UnknownValueError:
            print("Oops! Unable to understand the audio input.")
        except sr.RequestError as e:
            print(
                "Oops! Could not request results from Google Speech Recognition service; {0}".format(e))
            
        return item

    def keywordCheck(self, audio):
        text = self.recognizer.recognize_google(audio)
        command = self.nlp(text)
        item = 'nothing'
        for token in command:
            print(token, token.idx)

            # needs to be refactored to return aruco ID from dictionary
            if token.text == "water":
                print("water")
                item = token.text
            elif token.text == "pill":
                print("pill")
                item = token.text
            elif token.text == "nuts":
                print("nuts")
                item = token.text
            elif token.text == "fruits":
                print("fruits")
                item = token.text
            elif token.text == "coffee":
                print("coffee")
                item = token.text
            elif token.text == "thank":
                print("thank you")
                item = token.text
            else:
                item = "unknown"
                print("I did not catch any keywords")

        print (item)

        return item


if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('speech_node', anonymous=True)

    # Create a talker object and introduce yourself
    talker = Talker()
    talker.talk('Hello friends my name is TIAGo. I can retrieve objects for you through verbal requests. Some examples include water bottles, pill bottles, coffee, mixed nuts, and dried fruits.', language='en_GB', block=True)

    # Create recognizer object
    r = sr.Recognizer()
    nlp = spacy.load("en_core_web_sm")
    Listener.listen()
