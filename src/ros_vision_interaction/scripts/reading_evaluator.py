#!/usr/bin/env python3.8
import datetime
import glob
import os
import pymongo
import rospy
import speech_recognition
import sys
import time
import webrtcvad

from pydub import AudioSegment, silence
from std_msgs.msg import Bool
from vision_project_tools import init_db
from vision_project_tools import EngineStateDb as StateDb
from controllers.vision_project_delegator import INITIAL_STATE_DB

sys.path.append('/root/catkin_ws/src/ffmpeg')


class ReadingEvaluator:

    def __init__(
            self,
            statedb,
            upload_directory,
            file_prefix,
            extension="wav",
            silence_threshold=-35,
            chunk_size=10,
            seconds_to_check_for_audio_files=5
    ):
        if not os.path.exists(upload_directory):
            raise FileNotFoundError("The upload directory \'{}\' does not exist".format(upload_directory))

        self._upload_directory = upload_directory
        self._file_prefix = file_prefix
        self._extension = extension
        self._silence_threshold = silence_threshold
        self._chunk_size = chunk_size
        self._seconds_to_check_for_audio_files = seconds_to_check_for_audio_files

        is_record_evaluation_topic = rospy.get_param("controllers/is_record/evaluation")
        self._is_record_subscriber = rospy.Subscriber(
            is_record_evaluation_topic,
            Bool,
            callback=self.reading_analyzer_callback,
            queue_size=1
        )

        self._state_database = statedb
        self._vad = webrtcvad.Vad()
        self._vad.set_mode(1)

    def reading_analyzer_callback(self, is_record):
        rospy.loginfo(is_record.data)
        if not is_record.data:
            rospy.loginfo("Evaluator callback starting")
            audio_file = self.find_audio_file()
            reading_time = self.get_length_of_trimmed_audio(audio_file)
            reading_eval_index = self._state_database.get("reading eval index")
            try:
                num_of_words = self._state_database.get("reading eval data")[reading_eval_index]["word count"]
            except IndexError or KeyError:
                rospy.logerr(f"Reading task data not found for index:{reading_eval_index}")
                num_of_words = 0
            try:
                reading_speed = num_of_words/reading_time
                rospy.loginfo("Reading speed: {}".format(reading_speed))
            except ZeroDivisionError:
                rospy.logerr("Reading time is 0, setting reading speed to 0")
                reading_speed = 0
            self._state_database.set("current eval score", reading_speed)

    def find_audio_file(self):
        list_of_audio_files = []
        for _ in range(self._seconds_to_check_for_audio_files):
            list_of_audio_files = glob.glob(os.path.join(self._upload_directory, "*." + self._extension))
            if len(list_of_audio_files) > 0 and \
                    any(self._file_prefix in string for string in list_of_audio_files):
                break
            time.sleep(0.2)
        for file in list_of_audio_files:
            if file.find(self._file_prefix) >= 0:
                return file

    def get_length_of_trimmed_audio(self, audio_file):
        if audio_file is None:
            rospy.loginfo("No audio file passed")
            return 0

        if not audio_file.endswith(self._extension):
            raise TypeError("Must be a {} file".format(self._extension))

        rospy.loginfo(f"File name: {audio_file}")
        audio_segment = AudioSegment.from_wav(audio_file)
        audio_segment = self._amplify_speech(audio_segment)

        silent_segments = silence.detect_silence(audio_segment, silence_thresh=self._silence_threshold)
        leading_silence_index = silent_segments[0][1]
        ending_silence_index = silent_segments[len(silent_segments)-1][0]
        rospy.loginfo(f"Leading silence index: {leading_silence_index}, ending silence index: {ending_silence_index}")

        trimmed_audio = audio_segment[leading_silence_index:ending_silence_index]
        file_path = f"/root/audio_test/trimmed_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.wav"
        trimmed_audio.export(
            file_path,
            format="wav"
        )
        self._analyze_loudness(trimmed_audio)
        rospy.loginfo(
            f"Original length: {audio_segment.duration_seconds}\nTrimmed length: {trimmed_audio.duration_seconds}"
        )
        return audio_segment.duration_seconds
    
    def _amplify_speech(self, audio_segment, chunk_size=10):
        start_index = 0
        duration = len(audio_segment)
        # while start_index < duration:
        #     if audio_segment[start_index:start_index+chunk_size]:
        #         audio_segment[start_index:start_index+chunk_size].apply_gain(20)
        #         start_index += chunk_size
        self._analyze_loudness(audio_segment)
        audio_segment = audio_segment.apply_gain(+10)
        file_path = f"/root/audio_test/test_{datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.wav"
        audio_segment.export(
            file_path,
            format="wav"
        )
        return audio_segment

    def _analyze_loudness(self, audio_segment, chunk_size=10):
        loudness_array = []
        start_index = 0
        duration = len(audio_segment)
        while start_index < duration:
            loudness_array.append(audio_segment[start_index:start_index+chunk_size].dBFS)
            start_index += chunk_size
        rospy.loginfo(loudness_array)


if __name__ == "__main__":
    rospy.init_node("reading_evaluator")
    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    init_db(state_database, INITIAL_STATE_DB)

    upload_directory = rospy.get_param("vision-project/data/data_capture")

    reading_evaluator = ReadingEvaluator(
        state_database,
        upload_directory=upload_directory,
        file_prefix="evaluation",
        extension="wav",
        silence_threshold=-40,
        chunk_size=10
    )

    rospy.spin()
