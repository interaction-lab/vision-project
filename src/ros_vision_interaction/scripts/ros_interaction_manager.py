#!/usr/bin/python3.8
import actionlib
import datetime
import json
import os
import pymongo
import rospy

from controllers import InteractionManager
from controllers.vision_project_delegator import INITIAL_STATE_DB
from interaction_builder import InteractionBuilder
from interfaces import CordialInterface
from vision_project_tools import init_db
from vision_project_tools.engine_statedb import EngineStateDb as StateDb

from ros_vision_interaction.msg import StartInteractionAction, StartInteractionResult
from std_msgs.msg import Bool, String


class RosInteractionManager:

    def __init__(
            self,
            interaction_manager,
            state_database,
    ):
        self._interaction_manager = interaction_manager
        self._state_database = state_database

        is_go_to_sleep_topic = rospy.get_param('cordial/sleep_topic')
        node_name_topic = rospy.get_param('controllers/node_name_topic')
        self._sleep_publisher = rospy.Publisher(is_go_to_sleep_topic, Bool, queue_size=1)
        self._node_name_publisher = rospy.Publisher(node_name_topic, String, queue_size=1)

        # set up action server
        start_interaction_action_name = rospy.get_param("controllers/is_start_interaction")
        self._start_interaction_action_server = actionlib.SimpleActionServer(
            start_interaction_action_name,
            StartInteractionAction,
            self.run_manager_once,
            auto_start=False
        )
        self._start_interaction_action_server.register_preempt_callback(self._preempt_callback)

        self._is_debug = rospy.get_param(
            "vision-project/controllers/is_debug",
            False
        )
        self._start_interaction_action_server.start()

    # start interaction action callback
    def run_manager_once(self, goal):
        self._state_database.set("is interaction finished", False)
        interaction_type = goal.type
        result = StartInteractionResult()

        if not self._is_debug:
            for node_name in self._interaction_manager.run_interaction_once(interaction_type):
                self._node_name_publisher.publish(node_name)
            rospy.loginfo("Interaction finished")
        else:
            seconds_to_sleep_for_tests = 3
            rospy.sleep(seconds_to_sleep_for_tests)
            result.is_interaction_successful = True

        if not self._start_interaction_action_server.is_preempt_requested():
            rospy.loginfo("Setting goal as succeeded")
            self._start_interaction_action_server.set_succeeded(result)

    def _preempt_callback(self):
        rospy.loginfo("Preempt requested for interaction server")
        self._start_interaction_action_server.set_preempted()


if __name__ == "__main__":

    rospy.init_node("interaction_manager")

    seconds_until_timeout = rospy.get_param("vision-project/params/seconds_until_interaction_timeout")

    DATABASE_NAME = "vision-project"
    host = rospy.get_param("mongodb/host")
    port = rospy.get_param("mongodb/port")
    state_database = StateDb(
        pymongo.MongoClient(host, port),
        database_name=DATABASE_NAME,
        collection_name="state_db"
    )
    init_db(state_database, INITIAL_STATE_DB)

    # set up resources paths
    cwd = os.path.dirname(os.path.abspath(__file__))
    resources_directory = os.path.join(cwd, '..', 'resources')

    deployment_interaction_file = os.path.join(resources_directory, 'deployment', 'test_interactions.json')

    interaction_variations_file = os.path.join(resources_directory, 'deployment', 'interaction_variations.json')
    grit_dialogue_variations = os.path.join(resources_directory, 'deployment', 'grit_dialogue.json')

    max_num_of_perseverance_readings = rospy.get_param("vision-project/params/max_num_of_perseverance_readings")

    with open(deployment_interaction_file) as f:
        deployment_interaction_dict = json.load(f)

    interface = CordialInterface(
        state_database,
        seconds_until_timeout=seconds_until_timeout
    )

    interaction_builder = InteractionBuilder(
        interaction_dict=deployment_interaction_dict,
        variations_files=interaction_variations_file,
        statedb=state_database
    )

    interaction_manager = InteractionManager(
        statedb=state_database,
        interaction_builder=interaction_builder,
        interface=interface,
        max_num_of_perseverance_readings=max_num_of_perseverance_readings
    )

    ros_interaction_manager = RosInteractionManager(
        interaction_manager,
        state_database,
    )

    rospy.spin()
