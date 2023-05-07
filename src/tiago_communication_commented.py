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

        # A mapping from keywords or phrases to commands
        self.keywords_to_ID = {'water': ['water'],
                               'pill': ['pill'],
                               'coffee': ['coffee'],
                               'fruits': ['fruits'],
                               'nuts': ['nuts']}

        self.recognizer = sr.Recognizer()
        self.nlp = spacy.load("en_core_web_sm")

    """
    Function to listen to the audio input of the user.

    This function will wait for the user to give in audio input and then calls another function
    to determine the user input.

        Args:
            self: passes the object calling the function as a parameter.
        Returns:
            the item that the user asked tiago to fetch.

    """
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

    """
    Function to determine what the user input is.

    This function compares the user audio input to the set keywords that we defined and prints 
    the determined keyword of the item. It returns this item to the code for telling the 
    Tiago which item to fetch.

        Args:
            audio: the audio text input from the user that the Tiago recorded.
        Returns:
            the item that was determined from the audio input.
    """
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
            elif token.text == "vitamins":
                print("vitamins")
                item = token.text
            elif token.text == "oats":
                print("oats")
                item = token.text
            elif token.text == "thank":
                print("thank you")
                item = token.text
            else:
                item = "unknown"
                print("I did not catch any keywords")

        print (item)

        return item

"""
Main function to run and give some social interactive cues to Tiago with the user.

It will create instances of the talker class and use them to tell the Tiago what 
phrase to speak. We also called the listener functions to recognise what the user is
saying to the Tiago.

"""
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
