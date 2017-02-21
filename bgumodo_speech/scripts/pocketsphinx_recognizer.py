#!/usr/bin/env python

# rosrun komodo_speech pocketsphinx_recognizer.py

# [INFO] [WallTime: 1487692710.010623] [2125.182000] DETECTED: <s> KOMODO(2) BRING ME COFFEE(2) </s>
# [INFO] [WallTime: 1487692710.011259] [2125.223000] Command detected: coffee

import rospy

from pocketsphinx.pocketsphinx import *
# http://askubuntu.com/questions/691109/how-do-i-install-ffmpeg-and-codecs
import ffmpy

import os
import pyaudio
import wave
import audioop
from collections import deque
import time
import math

import commandrecognizer as cr

__author__ = 'dan'

class SpeechDetector:
    NODE_NAME = "pocketsphinx_recognizer"
    PACKAGE_NAME = "komodo_speech"

    debugLevel = rospy.DEBUG

    def __init__(self):
        fname = self.NODE_NAME
        rospy.init_node(self.NODE_NAME, anonymous=False, log_level=self.debugLevel)

        rospy.loginfo("{}: Initializing pocketsphinx_recognizer node".format(fname))

        # Microphone stream config.
        self.CHUNK = 8192#1024  # CHUNKS of bytes to read each time from mic
        self.FORMAT = pyaudio.paInt16
        self.CHANNELS = 1
        self.RECORD_RATE = 44100 # The default rate of the microphone
        self.POCKET_SPHINX_RATE = 16000 # Must be 16000 and not 44100 in order for pocketsphinx to work!
        # FFMPEG is used to convert from 44100 to 16000:
        self.FFMPEG_PATH = 'ffmpeg'

        self.SILENCE_LIMIT = 1  # Silence limit in seconds. The max ammount of seconds where
                           # only silence is recorded. When this time passes the
                           # recording finishes and the file is decoded

        self.PREV_AUDIO = 0.5  # Previous audio (in seconds) to prepend. When noise
                          # is detected, how much of previously recorded audio is
                          # prepended. This helps to prevent chopping the beginning
                          # of the phrase.

        self.THRESHOLD = 4500
        self.num_phrases = -1
        self.script_path = '/home/dan/catkin_ws/src/bgumodo-dev/bgumodo_speech/scripts/'
        self.lm = self.script_path + '../dict/cmd.lm.bin'
        self.dic = self.script_path + '../dict/cmd.dic'

        # These will need to be modified according to where the pocketsphinx folder is
        MODELDIR = "/home/dan/pocketsphinx/model"

        # Create a decoder with certain model
        config = Decoder.default_config()
        config.set_string('-hmm', os.path.join(MODELDIR, 'en-us/en-us'))
        #config.set_string('-lm', os.path.join(MODELDIR, 'en-us/en-us.lm.bin'))
        #config.set_string('-dict', os.path.join(MODELDIR, 'en-us/cmudict-en-us.dict'))
        config.set_string('-lm', self.lm)
        config.set_string('-dict', self.dic)

        # Creaders decoder object for streaming data.
        self.decoder = Decoder(config)


    def setup_mic(self, num_samples=10):
        """ Gets average audio intensity of your mic sound. You can use it to get
            average intensities while you're talking and/or silent. The average
            is the avg of the .2 of the largest intensities recorded.
        """
        rospy.loginfo("Getting intensity values from mic.")
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RECORD_RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)

        values = [math.sqrt(abs(audioop.avg(stream.read(self.CHUNK), 4)))
                  for x in range(num_samples)]
        values = sorted(values, reverse=True)
        r = sum(values[:int(num_samples * 0.2)]) / int(num_samples * 0.2)
        rospy.loginfo(" Finished ")
        rospy.loginfo(" Average audio intensity is " + str(r))
        stream.close()
        p.terminate()

        # if r < 3000:
        #     self.THRESHOLD = 3500
        # else:
        #     self.THRESHOLD = r + 100

        self.THRESHOLD = r + 100

        rospy.loginfo('Threshold:' + str(self.THRESHOLD))

    def save_speech(self, data, p):
        """
        Saves mic data to temporary WAV file. Returns filename of saved
        file
        """
        filename = self.script_path + 'output_'+str(int(time.time()))
        temp_filename = filename + '_temp.wav'
        # writes data to WAV file
        data = ''.join(data)
        wf = wave.open(temp_filename, 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(self.RECORD_RATE)
        wf.writeframes(data)
        wf.close()

        ff = ffmpy.FFmpeg(executable=self.FFMPEG_PATH,
                          inputs={temp_filename: None},
                          outputs={filename + '.wav': '-ar ' + str(self.POCKET_SPHINX_RATE) + ' -ac 1'})
        ff.run()

        os.remove(temp_filename)

        return filename + '.wav'

    def decode_phrase(self, wav_file):
        self.decoder.start_utt()
        stream = open(wav_file, "rb")
        while True:
          buf = stream.read(1024)
          if buf:
            self.decoder.process_raw(buf, False, False)
          else:
            break
        self.decoder.end_utt()
        words = []
        [words.append(seg.word) for seg in self.decoder.seg()]
        return words

    def run(self):
        """
        Listens to Microphone, extracts phrases from it and calls pocketsphinx
        to decode the sound
        """


        self.setup_mic()

        #Open stream
        p = pyaudio.PyAudio()
        stream = p.open(format=self.FORMAT,
                        channels=self.CHANNELS,
                        rate=self.RECORD_RATE,
                        input=True,
                        frames_per_buffer=self.CHUNK)
        rospy.loginfo("* Mic set up and listening. ")

        audio2send = []
        cur_data = ''  # current chunk of audio data
        rel = self.RECORD_RATE/self.CHUNK
        slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
        #Prepend audio from 0.5 seconds before noise was detected
        prev_audio = deque(maxlen=self.PREV_AUDIO * rel)
        started = False

        while True:
            cur_data = stream.read(self.CHUNK)
            slid_win.append(math.sqrt(abs(audioop.avg(cur_data, 4))))

            if sum([x > self.THRESHOLD for x in slid_win]) > 0:
                if started == False:
                    rospy.loginfo("Starting recording of phrase")
                    started = True
                audio2send.append(cur_data)

            elif started:
                rospy.loginfo("Finished recording, decoding phrase")
                filename = self.save_speech(list(prev_audio) + audio2send, p)
                recognized_words = self.decode_phrase(filename)
                recognized_text = ' '.join(recognized_words)
                rospy.loginfo("DETECTED: " + recognized_text)
                command = cr.recognize_command(recognized_text)
                if command:
                    rospy.loginfo("Command detected: " + command)
                else:
                    rospy.loginfo("Command detected: None")

                # Removes temp audio file
                os.remove(filename)
                # Reset all
                started = False
                slid_win = deque(maxlen=self.SILENCE_LIMIT * rel)
                prev_audio = deque(maxlen=0.5 * rel)
                audio2send = []
                rospy.loginfo("Listening ...")

            else:
                prev_audio.append(cur_data)

        rospy.loginfo("* Done listening")
        stream.close()
        p.terminate()

def main():
    sd = SpeechDetector()
    sd.run()

    #rospy.spin()

if __name__ == '__main__':
    main()
