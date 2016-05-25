#!/usr/bin/env python
import traceback
import os
import re

import recognizer_handler as RH

import rospkg
import rospy
from komodo_speech.msg import KomodoSpeechSayCommand
from komodo_speech.msg import KomodoSpeechSayResult
from komodo_speech.msg import KomodoSpeechRecCommand
from komodo_speech.msg import KomodoSpeechRecResult
from komodo_speech.srv import KomodoSpeechProcessing
from komodo_speech.srv import KomodoSpeechProcessingResponse
from sound_play.libsoundplay import SoundClient

__author__ = 'neowizard'


class KomodoSpeech:
    NODE_NAME = "KomodoSpeech"
    PACKAGE_NAME = "komodo_speech"

    debugLevel = rospy.DEBUG

    inProgress = False

    rec_handlers = {}

    soundClient = None
    sayCommandSub = None
    sayResultPub = None
    speechRecPub = None
    speechRecSub = None
    isInProgressSrv = None

    def __init__(self, voice='voice_cmu_us_rms_arctic_clunits'):
        fname = self.NODE_NAME
        rospy.init_node(self.NODE_NAME, anonymous=False, log_level=self.debugLevel)

        rospy.loginfo("{}: Initializing speech node".format(fname))

        # Set up speech synthesis
        self.soundClient = SoundClient()
        self.soundClientVoice = voice

        # Set up speech recognition
        self.init_recognizers()

        # Set up command listeners
        self.sayCommandSub = \
            rospy.Subscriber("~say_command", KomodoSpeechSayCommand, self.say_cb, queue_size=2)
        self.speechRecSub = \
            rospy.Subscriber("~rec_command", KomodoSpeechRecCommand, self.recognize_cb)

        # Set up result publishers
        self.sayResultPub = rospy.Publisher("~say_result", KomodoSpeechSayResult, queue_size=2)
        self.speechRecPub = rospy.Publisher("~speech_rec_result", KomodoSpeechRecResult, queue_size=2)

        # Set up utility services
        self.isInProgressSrv = \
            rospy.Service("~isProcessing", KomodoSpeechProcessing, self.is_running_cb)

        rospy.loginfo("{}: Speech module ready".format(fname))

    def init_recognizers(self):
        fname = "{}::{}".format(self.NODE_NAME, self.init_recognizers.__name__)

        # Get paths relative to ROS package path
        package_path = rospkg.RosPack().get_path(self.PACKAGE_NAME)
        dict_dir = package_path + "/dict"
        log_dir = package_path + "/logs"

        # Every corpus in the dict directory has its own handler (different category)
        for corpus_file in (dir_file for dir_file in os.listdir(dict_dir) if dir_file.endswith(".corpus")):
            category = corpus_file.replace(".corpus", "")
            if (os.path.exists(dict_dir + "/" + category + ".dic") and
                    os.path.exists(dict_dir + "/" + category + ".lm")):
                match_regexs_ids = self.get_corpus_match_re(dict_dir + "/" + category + ".match")
                lm_path = dict_dir + "/" + category + ".lm"
                dic_path = dict_dir + "/" + category + ".dic"

                self.rec_handlers[category] = RH.RecognizerHandler(category, match_regexs_ids,
                                                                   self.recognizer_match_cb, lm_path, dic_path, log_dir)
            else:
                rospy.logwarn("{}: Failed to find .lm or .dic file in {} for {} category"
                              .format(fname, dict_dir, category))

        for handler in self.rec_handlers.values():
            handler.wait_for_recognizer(10)

    def recognizer_match_cb(self, recognizer_handler, recognized_utterance, match_id_pair):
        fname = "{}::{}".format(self.NODE_NAME, self.recognizer_match_cb.__name__)

        rospy.loginfo("{}: Detected {}, Matched to {} (ID = {}) by the {} recognizer"
                      .format(fname, recognized_utterance, match_id_pair[0], match_id_pair[1],
                              recognizer_handler.recognizerId))

        recognizer_handler.stop_recognition()
        self.speechRecPub.publish(KomodoSpeechRecResult(success=True,
                                                        cat=recognizer_handler.recognizerId,
                                                        phrase_id=match_id_pair[1]))

    def get_corpus_match_re(self, corpus_file_path):
        fname = "{}::{}".format(self.NODE_NAME, self.get_corpus_match_re.__name__)

        curr_id = -1
        regexs_ids = {}
        with open(corpus_file_path) as corpus_file:
            for format_line in corpus_file:
                format_line = format_line.split('#')[0]  # Remove any trailing comment
                format_line = format_line.strip()  # Remove any leading/trailing newlines/tabs/spaces
                if (len(format_line) == 0):
                    continue
                elif (format_line.startswith("-ID")):
                    id_parts = format_line.split()
                    if (len(id_parts) != 2 or not id_parts[1].isdigit()):
                        rospy.logerr("{}: Invalid ID-tag format: {} (should be \"-ID <id>\"). Keeping curr ID, {}"
                                     .format(fname, format_line, curr_id))
                    else:
                        curr_id = int(id_parts[1])
                else:
                    regex = re.compile(format_line)
                    regexs_ids[regex] = curr_id

        return regexs_ids

    def recognize_cb(self, rec_command):
        fname = "{}::{}".format(self.NODE_NAME, self.recognize_cb.__name__)

        prev_state = self.inProgress
        self.inProgress = True

        rospy.logdebug("{}: Handling recognition command\n"
                       "\tCommand = {}\n"
                       "\tCAT = {}\n"
                       "\t".format(fname, rec_command.cmd, rec_command.cat))

        category = rec_command.cat

        if (rec_command.cmd == KomodoSpeechRecCommand.CMD_STOP):
            self.rec_handlers[category].stop_recognition()
            self.inProgress = prev_state
            return

        elif (rec_command.cmd == KomodoSpeechRecCommand.CMD_START):
            if not self.rec_handlers[category].start_recognition():
                rospy.logerr("{}: Can't perform speech recognition. "
                             "Failed to start PocketSphinx recognizer {}".
                             format(fname, self.rec_handlers[category].RecognizerID))
                self.inProgress = prev_state
        else:
            rospy.logdebug("{}: Unknown recognition command {}".format(fname, rec_command.cmd))

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


def main():
    roomScanner = KomodoSpeech()
    rospy.spin()

if __name__ == '__main__':
    main()
