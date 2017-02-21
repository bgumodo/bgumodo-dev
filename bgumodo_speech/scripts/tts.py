#!/usr/bin/env python

import rospy
import pyttsx

__author__ = 'dan'

class TTS:
    NODE_NAME = "tts"
    PACKAGE_NAME = "komodo_speech"

    debugLevel = rospy.DEBUG

    def __init__(self):
        fname = self.NODE_NAME
        rospy.init_node(self.NODE_NAME, anonymous=False, log_level=self.debugLevel)

        rospy.loginfo("{}: Initializing tts node".format(fname))

    def say_text(self, text):
        engine = pyttsx.init()
        rate = engine.getProperty('rate')
        engine.setProperty('rate', rate - 30)
        engine.say(text)
        engine.runAndWait()

def main():
    tts = TTS()
    tts.say_text('Hi there!')

    # rospy.spin()

if __name__ == '__main__':
    main()