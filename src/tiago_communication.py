"""
Tiago-Communication - ROSPY code for text-to-speech and speech-to-text conversion for the Tiago.

In this file, we have created the text-to-speech and speech-to-text code that the tiago uses
for communication with the user. It calls objects which listens and sends commands to the Tiago
to speak. 

This file uses speech recognition library of python along with TtsAction and TtsGoal libraries
of pal_interaction_msgs.msg to communicate the TTS phrases.

Usage:
    1. Import the necessary modules:
        import rospy
        import time
        import speech_recognition as sr
        import spacy

        from actionlib import SimpleActionClient
        from pal_interaction_msgs.msg import TtsAction, TtsGoal

    2. Create an instance of the robot_body class :
        talker = Talker()
        
    3. Call the various functions of the class using the created instance in main.py:
        item = listener.listen()

    4. Run the node using the main() function in main.py:
        if __name__ == '__main__':
            HRI_Script()
"""

#!/usr/bin/env python

import rospy
import time
import speech_recognition as sr
import spacy

from actionlib import SimpleActionClient
from pal_interaction_msgs.msg import TtsAction, TtsGoal

"""
Initialise function for the speech node.

This function starts the speech node of ROS for our communication.

"""

def init_ros():
    rospy.init_node('speech_node', anonymous=False)


class Talker:

    """
    Initialisation function for the basic setup of the Talker class.

    This function will set the language requirement of the Tiago and start TtsAction node
    of the Tiago for it's communication.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            no data is returned in this function.
    """

    def __init__(self):
        self.language = 'en_GB'

        # Connect to text-to-speech action server
        self.ac = SimpleActionClient('/tts', TtsAction)
        print('waiting for tts server')
        self.ac.wait_for_server()
        print('Initialised talker')

    """
    Function to get the tiago to talk phrases.

    This function takes in the input text from the function that calls it and sends the text 
    to the Tiago for verbal output.

        Args:
            text: the output phrase to be spoken.
            block (default=True): the decision to either talk and then wait or only talk 
                without waiting. By default, it will talk and then wait for input.
        Returns:
            no data is returned from this function.
    """

    def talk(self, text, block=True):
        goal = TtsGoal()
        goal.rawtext.lang_id = self.language
        goal.rawtext.text = text

        if block:
            self.ac.send_goal_and_wait(goal)
        else:
            self.ac.send_goal(goal)


class Listener:

    """
    Initialisation function for setup of the listener object.

    This function will do the basic setup of the Tiago, like setting the keywords, 
    starting the speech recognition node of the Tiago, and setting the natural language
    processing model that uses spaCy.

    """

    def __init__(self):

        #list to store keyword - aruco ID - item correspondence
        self.keyword_to_id = {
                              'pill': [12, 'pill bottle'],
                              'water': [8, 'water bottle'],
                              'oats': [13, 'oats jar'],
                              'nuts': [10, 'mixed nuts jar'],
                              'vitamins': [9, 'multi-vitamins'],
                              'thank': [-1,'thank you'],
                              'thanks': [-1,'thank you'],
                              'nothing': [-2, 'nothing']
                              }
        self.recognizer = sr.Recognizer()
        self.nlp = spacy.load("en_core_web_sm")
        print('Initialised listener')

    """
    Function to listen to the audio input of the user.

    This function will wait for the user to give in audio input. After it records an input, 
    it will try to determine what was spoken by the user by comparing it to our stored keywords
    list using Google Speech Recognition. If it finds a match, it goes ahead and tries to find 
    that object, or else it asks the user to speak again. 

    The TiaGo listens for a total of 20 seconds each time for a user input. 

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            the item name that the user asked tiago to fetch.

    """

    def listen(self):
        
        # Use default microphone as audio source
        item = 'nothing'
        with sr.Microphone() as source:

            # adjust for noise
            self.recognizer.adjust_for_ambient_noise(source)

            # prompt user to say something
            print("Please give me a command")

            # listen for user input
            print("Listening for 20sec")
            audio = self.recognizer.listen(source, timeout=10, phrase_time_limit=20)

            print("Heard something")

        try:
            # recognize speech using Google Speech Recognition
            audioText = self.recognizer.recognize_google(audio)
            print("Google Speech Recognition thinks you said " + audioText)
            command = self.nlp(audioText)
            #scan for keywords in list
            for com in command:
                print("Token text: " + com.text.lower(),
                      "  Token index: " + str(com.idx))
                
                #if a keyword from the list is found in the raw text, break
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
