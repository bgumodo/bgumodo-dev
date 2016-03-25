#!/usr/bin/env python
import traceback
import os
import re

import rospkg

import rospy
from komodo_speech.msg import KomodoSpeechSayCommand
from komodo_speech.msg import KomodoSpeechSayResult
from komodo_speech.msg import KomodoSpeechRecCommand
from komodo_speech.msg import KomodoSpeechRecResult
from komodo_speech.srv import KomodoSpeechProcessing
from komodo_speech.srv import KomodoSpeechProcessingResponse
from scripts.recognizer_handler import RecognizerHandler
from sound_play.libsoundplay import SoundClient

__author__ = 'neowizard'


class KomodoSpeech:
    NODE_NAME = "KomodoSpeech"
    PACKAGE_NAME = "komodo_speech"
    FORMATS_SEPARATOR = "----------"

    debugLevel = 1

    inProgress = False

    recognizers = {}

    soundClient = None
    sayCommandSub = None
    sayResultPub = None
    speechRecPub = None
    speechRecSub = None
    isInProgressSrv = None

    def __init__(self, voice='voice_cmu_us_rms_arctic_clunits'):
        fname = self.NODE_NAME

        if (self.debugLevel >= 1):
            rospy_debug_level = rospy.DEBUG
        else:
            rospy_debug_level = None
        rospy.init_node(self.NODE_NAME, anonymous=False, log_level=rospy_debug_level)

        rospy.loginfo("{}: Initializing speech node".format(fname))

        # Set up speech synthesis
        self.soundClient = SoundClient()
        self.soundClientVoice = voice

        # Set up speech recognition
        self.init_recognizers()

        self.sayResultPub = rospy.Publisher("~say_result", KomodoSpeechSayResult, queue_size=2)
        self.speechRecPub = rospy.Publisher("~speech_rec_result", KomodoSpeechRecResult, queue_size=2)

        self.sayCommandSub = \
            rospy.Subscriber("~say_command", KomodoSpeechSayCommand, self.say_cb, queue_size=2)
        self.speechRecSub = \
            rospy.Subscriber("~rec_command", KomodoSpeechRecCommand, self.recognize_cb)

        self.isInProgressSrv = \
            rospy.Service("~isProcessing", KomodoSpeechProcessing, self.is_running_cb)

        rospy.loginfo("{}: Speech module ready".format(fname))

    def init_recognizers(self):
        fname = "{}::{}".format(self.NODE_NAME, self.init_recognizers.__name__)

        dict_dir = rospkg.RosPack().get_path(self.PACKAGE_NAME) + "/dict"
        for corpus_file in (dir_file for dir_file in os.listdir(dict_dir) if dir_file.endswith(".corpus")):
            corpus = corpus_file.replace(".corpus", "")
            if (os.path.exists(dict_dir + "/" + corpus + ".dic") and
                    os.path.exists(dict_dir + "/" + corpus + ".lm")):
                match_re = self.get_corpus_match_re(dict_dir + "/" + corpus_file)
                lm_path = dict_dir + "/" + corpus + ".lm"
                dic_path = dict_dir + "/" + corpus + ".dic"

                self.recognizers[corpus] = \
                    RecognizerHandler(corpus, match_re, self.recognizer_match_cb, lm_path, dic_path)
            else:
                rospy.logwarn("{}: Failed to find .lm or .dic file in {} for {} corpus".format(fname, dict_dir, corpus))

    def recognizer_match_cb(self, recognizer_handler, matched_utterance, match_re):
        recognizer_handler.stoprecognition()
        self.speechRecPub.publish(KomodoSpeechRecResult(success=True, phrase=matched_utterance))



    def get_corpus_match_re(self, corpus_file_path):
        with open(corpus_file_path) as corpus_file:
            for format_line in corpus_file:
                if (format_line == self.FORMATS_SEPARATOR):
                    break
                format_re = format_line.replace(" ", ".*")
                yield re.compile(format_re)

    def recognize_cb(self, rec_command):
        fname = "{}::{}".format(self.NODE_NAME, self.recognize_cb.__name__)

        self.inProgress = True
        category = rec_command.cat

        if (rec_command.cmd == KomodoSpeechRecCommand.CMD_STOP):
            self.recognizers[category].stop_recogniion()
            self.inProgress = False
            return

        if (rec_command.cmd == KomodoSpeechRecCommand.CMD_START):
            if not self.recognizers[category].start_recognition():
                rospy.logerr("{}: Can't perform speech recognition. "
                             "Failed to start PocketSphinx recognizer {}".
                             format(fname, self.recognizers[category].RecognizerID))
                self.inProgress = False

    def is_running_cb(self):
        return KomodoSpeechProcessingResponse(self.inProgress)

    def say_cb(self, say_command_data):
        success = self.say_text(say_command_data.text_to_say)

        say_result = KomodoSpeechSayResult()
        say_result.success = success
        say_result.said_text = say_command_data.text_to_say
        self.sayResultPub.publish(say_result)

    def say_text(self, text):
        fname = "{}::{}".format(self.NODE_NAME, self.say_text.__name__)

        self.inProgress = True

        try:
            if (text == ""):
                rospy.logdebug("{}: Shutting up".format(fname))
                self.soundClient.stopAll()
            else:
                rospy.logdebug("{}: Saying \"{}\"...".format(fname, text))
                self.soundClient.say(text, self.soundClientVoice, blocking=True)

        except AssertionError as e:
            rospy.logfatal("{}: sound_play error:\n{}\n\n. Shutting Down...".format(fname, traceback.format_exc()))
            rospy.signal_shutdown("sound_play error")

        self.inProgress = False

        return True


if __name__ == '__main__':
    try:
        roomScanner = KomodoSpeech()
        rospy.spin()

    except rospy.ROSInterruptException, e:
        pass
