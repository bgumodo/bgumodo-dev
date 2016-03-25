#!/usr/bin/env python
import os
import rospy
import re
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty

__author__ = 'neowizard'


class RecognizerHandler:
    FORMATS_SEPARATOR = "----------"

    lmFile = ""
    dictFile = ""
    corpusPath = ""
    recognizerId = ""

    stopRecSvc = None
    startRecSvc = None

    recognizable_utterances_re = []
    match_cb = None

    lastRecOutput = ""
    recOutputListener = None

    def __init__(self, recognizer_id, match_re, match_cb, lm_file, dict_file):
        fname = "RecognizerHandler({})".format(recognizer_id)

        self.recognizerId = recognizer_id
        self.recognizerName = "recognizer_{}".format(self.recognizerId)
        self.lmFile = lm_file
        self.dictFile = dict_file
        self.match_cb = match_cb
        self.recognizable_utterances_re = match_re

        rospy.loginfo("{}: Initializing {} recognizer".format(fname, self.recognizerId))

        # TODO: replace with roslaunch API
        os.system("rosrun pocketsphinx recognizer.py {} &".format(recognizer_id))

        self.pocketSphinxStartSrv = rospy.ServiceProxy("/{}/start".format(self.recognizerName), Trigger)
        self.pocketSphinxStopSrv = rospy.ServiceProxy("/{}/stop".format(self.recognizerName), Empty)
        self.pocketSphinxStartSrv.wait_for_service(10)  # Give the recognizer 10sec to fully start up
        self.pocketSphinxStopSrv.wait_for_service(1)

        rospy.set_param("/{}/lm".format(self.recognizerName), self.lmFile)
        rospy.set_param("/{}/dict".format(self.recognizerName), self.dictFile)

        rospy.loginfo("{}: Recognizer initialized".format(fname))

    def stop_recognition(self):
        fname = "Handler({})::{}".format(self.recognizerName, self.stop_recognition.__name__)
        rospy.loginfo("{}: Stopping recognizer...".format(fname))

        self.stopRecSvc()
        self.recOutputListener.unregister()

    def start_recognition(self):
        fname = "Handler({})::{}".format(self.recognizerName, self.start_recognition.__name__)
        rospy.loginfo("{}: Starting recognizer...".format(fname))

        recognizer_started = self.startRecSvc()
        if not recognizer_started:
            rospy.logerr("{}: Failed to start PocketSphinx speech recognition.\n\t "
                         "This is usually a result of missing dict or lm files".format(fname))
            return False

        self.recOutputListener = \
            rospy.Subscriber("/{}/output".format(self.recognizerId), String, self.rec_output_cb, queue_size=2)

        return True

    def rec_output_cb(self, output_str):
        fname = "Handler({})::{}".format(self.recognizerId, self.rec_output_cb.__name__)
        self.lastRecOutput = output_str
        rospy.logdebug("{}: Last recognized utterance {}".format(fname, self.lastRecOutput))

        best_match = max(self.recognizable_utterances_re, key=lambda regex: len(regex.findall(output_str)))
        self.match_cb(self, output_str, best_match)



