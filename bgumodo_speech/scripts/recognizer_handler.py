#!/usr/bin/env python
import os
import rospy
import re
from std_msgs.msg import String
from std_srvs.srv import Empty

__author__ = 'neowizard'


class RecognizerHandler:
    FORMATS_SEPARATOR = "----------"

    recognizerLogDir = ""
    lmFile = ""
    dictFile = ""
    corpusPath = ""
    recognizerId = ""
    recognizerName = ""

    Running = False

    UtterancesRegexIds = {}
    match_cb = None

    lastRecStr = ""
    recOutputListener = None

    def __init__(self, recognizer_id, match_re, match_cb, lm_file, dict_file, recognizer_log_dir="."):
        fname = "RecognizerHandler({})".format(recognizer_id)

        self.recognizerLogDir = recognizer_log_dir
        self.recognizerId = recognizer_id
        self.recognizerName = "recognizer_{}".format(self.recognizerId)
        self.lmFile = lm_file
        self.dictFile = dict_file
        self.match_cb = match_cb
        self.UtterancesRegexIds = match_re

        rospy.loginfo("{}: Initializing {} recognizer (log stored at {}/{}.log)"
                      .format(fname, self.recognizerId, self.recognizerLogDir, self.recognizerId))

        rospy.set_param("/{}/lm".format(self.recognizerName), self.lmFile)
        rospy.set_param("/{}/dict".format(self.recognizerName), self.dictFile)
	#rospy.set_param("/{}/mic_name".format(self.recognizerName), "alsa_input.usb-PrimeSense_PrimeSense_Device-01-Device.iec958-stereo")

        # TODO: replace with roslaunch API
        os.system("bash -c \"rosrun komodo_speech recognizer.py {} &> {}/{}.log\" &"
                  .format(recognizer_id, self.recognizerLogDir, self.recognizerId))

    def wait_for_recognizer(self, timeout):
        fname = "Handler({})::{}".format(self.recognizerName, self.wait_for_recognizer.__name__)
        rospy.wait_for_service("/recognizer_{}/start".format(self.recognizerId), timeout)
        rospy.loginfo("{}: Recognizer ready".format(fname))

    def stop_recognition(self):
        fname = "Handler({})::{}".format(self.recognizerName, self.stop_recognition.__name__)

        if (not self.Running):
            rospy.logwarn("{}: Trying to stop an already stopped recognizer".format(fname))
            return

        rospy.loginfo("{}: Stopping recognizer...".format(fname))
        self.recOutputListener.unregister()
        self.Running = False

    def start_recognition(self):
        fname = "Handler({})::{}".format(self.recognizerName, self.start_recognition.__name__)

        if (self.Running):
            rospy.logwarn("{}: Attempting to start an already running recognizer".format(fname))
            return False

        rospy.loginfo("{}: Starting recognizer...".format(fname))

        self.recOutputListener = \
            rospy.Subscriber("/{}/output".format(self.recognizerName), String, self.rec_output_cb, queue_size=2)
        self.Running = True

        rospy.logdebug("{}: Started recognition".format(fname))

        return True

    def rec_output_cb(self, rec_output):
        fname = "Handler({})::{}".format(self.recognizerId, self.rec_output_cb.__name__)

        if (not self.Running):
            return

        rec_str = rec_output.data
        rospy.logdebug("{}: Got {} from recognizer. Attempting match".format(fname, rec_str))

        rec_regexs = self.UtterancesRegexIds.keys()
        best_match = max(rec_regexs, key=lambda regex: len(regex.findall(rec_str)))
        if (len(best_match.findall(rec_str)) > 0):
            self.lastRecStr = rec_str
            self.match_cb(self, self.lastRecStr, (best_match, self.UtterancesRegexIds[best_match]))
            rospy.logdebug("{}: Last matched utterance: {}".format(fname, self.lastRecStr))
        else:
            rospy.logdebug("{}: No match.".format(fname))
