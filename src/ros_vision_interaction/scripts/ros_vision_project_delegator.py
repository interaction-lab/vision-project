#!/usr/bin/env python3.8
import actionlib
import datetime
import pymongo
import rospy
import schedule
import threading

from controllers import VisionProjectDelegator
from controllers.vision_project_delegator import INITIAL_STATE_DB
from interaction_builder import Graphs
from vision_project_tools import init_db
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

from cordial_msgs.msg import MouseEvent
from ros_vision_interaction.msg import StartInteractionAction, StartInteractionGoal
from std_msgs.msg import Bool, String


class RosVisionProjectDelegator:

    def __init__(
            self,
            vision_project_delegator,
    ):
        self._minutes_between_interactions = datetime.timedelta(
            minutes=rospy.get_param("vision-project/controllers/minutes_between_interactions")
        )
        self._scheduled_time_window_minutes = datetime.timedelta(
            minutes=rospy.get_param("vision-project/controllers/scheduled_window_minutes")
        )

        self._delegator = vision_project_delegator
        self._state_database = state_database
        self._seconds_between_updates = rospy.get_param("vision-project/controllers/update_window_seconds")

        # action client to start interaction
        start_interaction_action_name = rospy.get_param("controllers/is_start_interaction")
        self._start_interaction_client = actionlib.SimpleActionClient(
            start_interaction_action_name,
            StartInteractionAction
        )

        # ROS publishers and subscribers
        is_record_interaction_topic = rospy.get_param("controllers/is_record/interaction")
        is_record_evaluation_topic = rospy.get_param("controllers/is_record/evaluation")
        is_record_perseverance_topic = rospy.get_param("controllers/is_record/perseverance")
        screen_tap_topic = rospy.get_param("cordial/screen_tap")
        node_name_topic = rospy.get_param('controllers/node_name_topic')
        self._is_record_interaction_publisher = rospy.Publisher(is_record_interaction_topic, Bool, queue_size=1)
        self._is_record_evaluation_publisher = rospy.Publisher(is_record_evaluation_topic, Bool, queue_size=1)
        self._is_record_perseverance_publisher = rospy.Publisher(is_record_perseverance_topic, Bool, queue_size=1)
        self._screen_tap_listener = rospy.Subscriber(
            screen_tap_topic,
            MouseEvent,
            callback=self._screen_tap_listener_callback,
            queue_size=1
        )
        self._node_name_subscriber = rospy.Subscriber(
            node_name_topic,
            String,
            callback=self._node_name_callback,
            queue_size=1
        )

        # update scheduler
        self._scheduler = schedule.Scheduler()
        self._scheduler.every(self._seconds_between_updates).seconds.do(self.update)

        self._current_node_name = None
        self._is_recording_evaluation = False
        self._is_recording_perseverance = False

        self._is_debug = rospy.get_param(
            "vision-project/controllers/is_debug",
            False
        )

    def run_scheduler_once(self):
        self._scheduler.run_pending()

    def update(self):
        rospy.loginfo("Running update")

        self._delegator.update()
        interaction_type = self._delegator.get_interaction_type()
        if interaction_type is not None:
            rospy.loginfo("Delegating interaction: {}".format(interaction_type))
            self.delegate_interaction(interaction_type)

    def delegate_interaction(self, interaction_type):
        self._start_interaction_client.wait_for_server()

        start_interaction_goal = StartInteractionGoal()
        start_interaction_goal.type = interaction_type

        if not self._is_debug:
            # TODO: add a feedback callback
            rospy.loginfo("Sending goal to start interaction")
            self._start_interaction_client.send_goal(start_interaction_goal)
            self._is_record_interaction_publisher.publish(True)

            self._start_interaction_client.wait_for_result()

            self._is_record_interaction_publisher.publish(False)
            if self._is_recording_evaluation:
                self._is_record_evaluation_publisher.publish(False)
                self._is_recording_evaluation = False
            if self._is_recording_perseverance:
                self._is_record_perseverance_publisher.publish(False)
                self._is_recording_perseverance = False

        return

    def _screen_tap_listener_callback(self, _):
        last_interaction_time = self._state_database.get("last interaction datetime")
        # TODO: change time btwn interactions to a few seconds
        if last_interaction_time is not None:
            enough_time_passed = datetime.datetime.now() - self._state_database.get("last interaction datetime") \
                                 > self._minutes_between_interactions
            if not enough_time_passed:
                rospy.loginfo("Not enough time passed to initiate an interaction")
        else:
            enough_time_passed = False

        if self._state_database.get("is interaction finished") and enough_time_passed:
            rospy.loginfo("is prompted by user: True")
            self._state_database.set("is prompted by user", True)

    def _node_name_callback(self, node_name):
        rospy.loginfo(node_name)
        if node_name == Graphs.EVALUATION:
            self._is_record_evaluation_publisher.publish(True)
            self._is_recording_evaluation = True
        if node_name == Graphs.PERSEVERANCE:
            self._is_record_perseverance_publisher.publish(True)
            self._is_recording_perseverance = True


if __name__ == "__main__":

    rospy.init_node("vision_project_delegator")

    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    init_db(state_database, INITIAL_STATE_DB)

    update_window_seconds = rospy.get_param("vision-project/controllers/update_window_seconds")
    scheduled_window_minutes = rospy.get_param("vision-project/controllers/scheduled_window_minutes")
    minutes_between_interactions = rospy.get_param("vision-project/controllers/minutes_between_interactions")
    max_num_of_prompted_per_day = rospy.get_param("vision-project/params/max_num_of_prompted_per_day")

    vision_project_delegator = VisionProjectDelegator(
        statedb=state_database,
        update_window_seconds=update_window_seconds,
        minutes_between_interactions=minutes_between_interactions,
        max_num_of_prompted_per_day=max_num_of_prompted_per_day
    )

    ros_vision_project_delegator = RosVisionProjectDelegator(vision_project_delegator)

    while not rospy.is_shutdown():
        ros_vision_project_delegator.run_scheduler_once()
        rospy.sleep(1)
