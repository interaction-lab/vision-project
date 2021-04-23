#!/usr/bin/env python3.8
import pymongo
import rospy

from std_msgs.msg import Bool
from vision_project_tools import AudioAnalyzer, init_db
from vision_project_tools import EngineStateDb as StateDb
from controllers.vision_project_delegator import INITIAL_STATE_DB


class ReadingEvaluator:

    def __init__(
            self,
            audio_trimmer,
            statedb
    ):
        self._audio_trimmer = audio_trimmer
        is_record_evaluation_topic = rospy.get_param("controllers/is_record/evaluation")
        self._is_record_subscriber = rospy.Subscriber(
            is_record_evaluation_topic,
            Bool,
            callback=self.is_done_recording_callback,
            queue_size=1
        )
        self._state_database = statedb

    def is_done_recording_callback(self, is_record):
        rospy.loginfo(is_record.data)
        if not is_record.data:
            rospy.loginfo("Evaluator callback starting")
            reading_time = self._audio_trimmer.get_length_of_trimmed_audio()
            reading_eval_index = self._state_database.get("reading eval index")
            try:
                num_of_words = self._state_database.get("reading eval data")[reading_eval_index]["word count"]
            except IndexError or KeyError:
                num_of_words = 0
            reading_speed = num_of_words/reading_time
            rospy.loginfo("Reading speed: {}".format(reading_speed))
            self._state_database.set("current eval score", reading_speed)


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
    copy_directory = rospy.get_param("vision-project/data/evaluation")
    audio_trimmer = AudioAnalyzer(
        upload_directory=upload_directory,
        copy_directory=copy_directory,
        file_prefix="evaluation",
        new_file_name="evaluation",
        extension="wav",
        silence_threshold=-20,
        chunk_size=10
    )

    reading_evaluator = ReadingEvaluator(audio_trimmer, state_database)

    rospy.spin()
