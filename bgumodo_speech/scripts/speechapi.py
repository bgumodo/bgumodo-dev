#!/usr/bin/env python

# Requires PyAudio and PySpeech.

# git clone http://people.csail.mit.edu/hubert/git/pyaudio.git
# cd pyaudio
# sudo apt-get install libportaudio-dev
# sudo apt-get install python-dev
# sudo apt-get install libportaudio0 libportaudio2 libportaudiocpp0 portaudio19-dev
# sudo python setup.py install
# sudo pip install SpeechRecognition

# sudo apt-get install jackd1

# install https://github.com/cmusphinx/sphinxbase
# install https://github.com/cmusphinx/pocketsphinx

import rospy

__author__ = 'dan'

import speech_recognition as sr

class SpeechAPI:
    NODE_NAME = "SpeechAPI"
    PACKAGE_NAME = "komodo_speech"

    debugLevel = rospy.DEBUG

    def __init__(self):
        fname = self.NODE_NAME
        rospy.init_node(self.NODE_NAME, anonymous=False, log_level=self.debugLevel)

        rospy.loginfo("{}: Initializing speech api node".format(fname))

        # Record Audio
        self.recognizer = sr.Recognizer()

        with sr.Microphone() as source:
            rospy.loginfo("Adjusting silence threshold...")
            self.recognizer.adjust_for_ambient_noise(source)
            rospy.loginfo("Done")

def main():
    speech_api = SpeechAPI()

    while not rospy.is_shutdown():
        with sr.Microphone() as source:
            rospy.loginfo("Say something!")
            audio = speech_api.recognizer.listen(source)
        try:
            rospy.loginfo("You said: " + speech_api.recognizer.recognize_sphinx(audio))
        except sr.UnknownValueError:
            rospy.logerr("Speech Recognition could not understand what you said")
        except sr.RequestError as e:
            rospy.logerr("RequestError: {0}".format(e))

    #rospy.spin()

if __name__ == '__main__':
    main()
