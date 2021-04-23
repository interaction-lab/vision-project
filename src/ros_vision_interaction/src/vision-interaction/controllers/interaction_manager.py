#!/usr/bin/python3.8
import datetime
import logging

from interaction_engine.interfaces import TerminalClientAndServerInterface
from interaction_engine.planner import MessagerPlanner
from interaction_builder import InteractionBuilder, Graphs
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine

logging.basicConfig(level=logging.INFO)


class Interactions:

    ASK_TO_DO_EVALUATION = "ask to do evaluation"
    EVALUATION = "evaluation"
    FIRST_INTERACTION = "first interaction"
    PROMPTED_INTERACTION = "prompted interaction"
    SCHEDULED_INTERACTION = "scheduled interaction"
    TOO_MANY_PROMPTED = "too many prompted"

    POSSIBLE_INTERACTIONS = [
        ASK_TO_DO_EVALUATION,
        EVALUATION,
        FIRST_INTERACTION,
        PROMPTED_INTERACTION,
        SCHEDULED_INTERACTION,
        TOO_MANY_PROMPTED
    ]


class InteractionManager:

    def __init__(
            self,
            statedb,
            interaction_builder,
            interface=None,
            max_num_of_perseverance_readings=5
    ):
        self._state_database = statedb

        self._interaction_builder = interaction_builder

        if interface is None:
            interface = TerminalClientAndServerInterface(database=self._state_database)
        self._interface = interface

        self._current_node_name = None

        self._planner = MessagerPlanner(self._interaction_builder.possible_graphs)

        self._max_num_of_perseverance_readings = max_num_of_perseverance_readings
        self._num_of_days_to_prompt_goal_setting = 3

    def run_interaction_once(self, interaction_type):
        if interaction_type not in Interactions.POSSIBLE_INTERACTIONS:
            raise ValueError("Not a valid interaction type")

        self.build_interaction(interaction_type)
        for node_name in self.run_engine_once():
            yield node_name

    def run_engine_once(self):
        engine = InteractionEngine(
            self._interface,
            self._planner,
            self._interaction_builder.possible_graphs
        )
        for node_name in engine.modified_run(self._planner):
            yield node_name

    def build_interaction(self, interaction_type):
        self._planner = MessagerPlanner(self._interaction_builder.possible_graphs)
        if interaction_type == Interactions.ASK_TO_DO_EVALUATION:
            self._build_ask_to_do_evaluation()
        elif interaction_type == Interactions.FIRST_INTERACTION:
            self._build_first_interaction()
        elif interaction_type == Interactions.PROMPTED_INTERACTION:
            self._build_prompted_interaction()
        elif interaction_type == Interactions.SCHEDULED_INTERACTION:
            self._build_scheduled_interaction()
        elif interaction_type == Interactions.TOO_MANY_PROMPTED:
            self._build_too_many_prompted()
        else:
            raise ValueError("Not a valid interaction type")

        return self._planner

    def _build_ask_to_do_evaluation(self):
        logging.info("Building ask to do scheduled")
        self._planner.insert(self._interaction_builder.interactions[Graphs.GREETING])
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.ASK_TO_DO_SCHEDULED],
            post_hook=self._set_vars_after_ask_for_eval,
        )
        return self._planner

    def _build_first_interaction(self):
        logging.info("Building first interaction")
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.FIRST_CHECKIN],
            post_hook=self._set_vars_after_first_interaction
        )
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.SCHEDULE_NEXT_CHECKIN],
            post_hook=self._set_vars_after_scheduling_next_checkin
        )
        return self._planner

    def _build_prompted_interaction(self):
        logging.info("Building prompted interaction")
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.GREETING]
        )
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.PROMPTED_CHECKIN],
            self._set_vars_after_prompted
        )
        return self._planner

    def _build_scheduled_interaction(self):
        logging.info("Building scheduled interaction")
        self._planner.insert(self._interaction_builder.interactions[Graphs.GREETING])
        # scheduled checkin should end w asking whether or not participant
        # wants to do the evaluation right now
        self._planner.insert(
            self._interaction_builder.interactions[Graphs.SCHEDULED_CHECKIN],
            post_hook=self._set_vars_after_scheduled
        )
        self._planner.insert(
            plan=self._interaction_builder.interactions[Graphs.CHECK_READING_ID]
        )
        self._planner.insert(
            plan=self._interaction_builder.interactions[Graphs.EVALUATION],
            post_hook=self._set_vars_after_evaluation
        )
        self._planner.insert(
            plan=self._interaction_builder.interactions[Graphs.ASK_TO_DO_PERSEVERANCE],
            post_hook=self._set_vars_after_ask_to_do_perseverance
        )
        return self._planner

    def _build_too_many_prompted(self):
        logging.info("Building checkin limit reminder")
        self._planner.insert(
            plan=self._interaction_builder.interactions[Graphs.TOO_MANY_PROMPTED],
            post_hook=self._set_vars_after_too_many_prompted
        )
        return self._planner

    def _set_vars_after_ask_for_eval(self):
        if self._state_database.get("is off checkin") == "Yes":
            self._planner.insert(
                self._interaction_builder.interactions[Graphs.CHECK_READING_ID]
            )
            self._planner.insert(
                self._interaction_builder.interactions[Graphs.EVALUATION],
                post_hook=self._set_vars_after_evaluation
            )
        else:
            self._planner.insert(
                self._interaction_builder.interactions[Graphs.PROMPTED_CHECKIN]
            )
        self._set_vars_after_interaction()

    def _set_vars_after_ask_to_do_perseverance(self):
        if self._state_database.get("is start perseverance") == "Yes":
            self._planner.insert(
                plan=self._interaction_builder.interactions[Graphs.PERSEVERANCE],
                post_hook=self._set_vars_after_perseverance
            )
        else:
            if self._is_do_mindfulness():
                self._planner.insert(
                    plan=self._interaction_builder.interactions[Graphs.MINDFULNESS],
                    post_hook=self._set_vars_after_mindfulness
                )
            if self._is_do_goal_setting():
                self._planner.insert(
                    plan=self._interaction_builder.interactions[Graphs.GOAL_SETTING],
                    post_hook=self._set_vars_after_goal_setting
                )
            self._planner.insert(
                plan=self._interaction_builder.interactions[Graphs.SCHEDULE_NEXT_CHECKIN]
            )

    def _set_vars_after_evaluation(self):
        self._state_database.set("is done eval today", True)
        eval_index = self._state_database.get("reading eval index")
        self._state_database.set("reading eval index", eval_index + 1)
        # also need to calculate and save reading speed
        self._set_vars_after_interaction()

    def _set_vars_after_interaction(self):
        self._state_database.set("is prompted by user", False)
        self._state_database.set("is interaction finished", True)
        self._state_database.set("last interaction datetime", datetime.datetime.now())

    def _set_vars_after_first_interaction(self):
        self._state_database.set("first interaction datetime", datetime.datetime.now())
        self._set_vars_after_interaction()

    def _set_vars_after_goal_setting(self):
        self._state_database.set("num of days since last goal setting", 0)
        self._set_vars_after_interaction()

    def _set_vars_after_mindfulness(self):
        self._state_database.set("num of days since last mindfulness", 0)
        self._set_vars_after_interaction()

    def _set_vars_after_perseverance(self):
        if self._state_database.get("perseverance counter") >= self._max_num_of_perseverance_readings or \
                self._state_database.get("is continue perseverance") != "Continue":
            self._planner.insert(
                plan=self._interaction_builder.interactions[Graphs.REWARD]
            )
            self._planner.insert(
                plan=self._interaction_builder.interactions[Graphs.SCHEDULE_NEXT_CHECKIN]
            )
        else:
            if self._state_database.get("is continue perseverance") == "Continue":
                self._planner.insert(
                    plan=self._interaction_builder.interactions[Graphs.PERSEVERANCE],
                    post_hook=self._set_vars_after_perseverance
                )
        perseverance_counter = self._state_database.get("perseverance counter") + 1
        self._state_database.set("perseverance counter", perseverance_counter)

    def _set_vars_after_prompted(self):
        num_of_prompted_today = self._state_database.get("num of prompted today") + 1
        self._state_database.set("num of prompted today", num_of_prompted_today)
        self._state_database.set("is prompted by user", False)
        self._state_database.set("is done prompted today", True)
        self._set_vars_after_interaction()

    def _set_vars_after_scheduled(self):
        self._state_database.set("is prompted by user", False)
        self._state_database.set("next checkin datetime", None)
        self._set_vars_after_interaction()

    def _set_vars_after_scheduling_next_checkin(self):
        self._set_vars_after_interaction()

    def _set_vars_after_too_many_prompted(self):
        self._set_vars_after_interaction()
        self._state_database.set("is prompted by user", False)

    def _is_do_mindfulness(self):
        last_5_scores = self._state_database.get("last 5 eval scores")
        if len(last_5_scores) > 0:
            average_eval_score = sum(last_5_scores) / len(last_5_scores)
        else:
            average_eval_score = 0
        return self._days_since_first_interaction() >= 7 \
            and self._state_database.get("feelings index") <= 3 \
            and self._state_database.get("current eval score") < average_eval_score

    def _is_do_goal_setting(self):
        last_5_scores = self._state_database.get("last 5 eval scores")
        if len(last_5_scores) > 0:
            average_eval_score = sum(last_5_scores)/len(last_5_scores)
        else:
            average_eval_score = 0
        return self._days_since_first_interaction() >= 7 \
            and self._state_database.get("feelings index") <= 3 \
            and self._state_database.get("num of days since last goal setting") >= 7 \
            and self._state_database.get("num of days since last prompt") >= self._num_of_days_to_prompt_goal_setting \
            and self._state_database.get("num of days since last perseverance") >= self._num_of_days_to_prompt_goal_setting \
            and self._state_database.get("current eval score") < average_eval_score

    def _days_since_first_interaction(self):
        first_interaction_datetime = self._state_database.get("first interaction datetime")
        if first_interaction_datetime is None:
            num_of_days = 0
        else:
            current_date = datetime.datetime.now().date()
            num_of_days = (current_date - self._state_database.get("first interaction datetime").date()).days
        return num_of_days

    @property
    def current_node_name(self):
        return self._current_node_name
