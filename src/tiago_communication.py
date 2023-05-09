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
        print('Initialised talker')

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

        self.keyword_to_id = {
                              'pill': [12, 'pill bottle'],
                              'water': [8, 'water bottle'],
                              'oats': [11, 'oats jar'],
                              'nuts': [10, 'mixed nuts jar'],
                              'vitamins': [9, 'multi-vitamins'],
                              'thank': [-1,'thank you'],
                              'thanks': [-1,'thank you'],
                              'nothing': [-2, 'nothing']
                              }
        self.recognizer = sr.Recognizer()
        self.nlp = spacy.load("en_core_web_sm")
        print('Initialised listener')

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
            audioText = self.recognizer.recognize_google(audio)
            print("Google Speech Recognition thinks you said " + audioText)
            command = self.nlp(audioText)
            for com in command:
                print("Token text: " + com.text.lower(),
                      "  Token index: " + str(com.idx))
                if self.keyword_to_id.get(com.text.lower()) != None:
                    item = com.text.lower()
                    print("Found item: " + item)
                    break
            else:
                print("Did not find any keywords")
                item = "unknown"

        except sr.UnknownValueError:
            print("Oops! Unable to understand the audio input.")
        except sr.RequestError as e:
            print(
                "Oops! Could not request results from Google Speech Recognition service; {0}".format(e))

        return item


if __name__ == '__main__':

    # Initialize this as a ROS node
    rospy.init_node('speech_node', anonymous=True)

    # Create a talker object and introduce yourself
    talker = Talker()
    talker.talk('Hello friends my name is TIAGo. I can retrieve objects for you through verbal requests. Some examples include water bottles, pill bottles, coffee, mixed nuts, and dried fruits.', language='en_GB', block=True)

    # Create recognizer object
    item = Listener.listen()
    print("I heard: " + item if item != 'nothing' else "I did not hear anything")
