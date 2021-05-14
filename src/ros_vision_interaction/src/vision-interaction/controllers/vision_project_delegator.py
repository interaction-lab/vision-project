#!/usr/bin/env python3.8
import datetime
import logging

from controllers.interaction_manager import Interactions

logging.basicConfig(level=logging.INFO)

INITIAL_STATE_DB = {
    "current eval score": 0,
    "current reading id": None,
    "first interaction datetime": None,
    "good to chat": None,
    "is continue perseverance": None,
    "is done eval today": False,
    "is done prompted today": False,
    "is done perseverance today": False,
    "is done mindfulness today": False,
    "is done goal setting today": False,
    "is interaction finished": False,
    "is off checkin": None,
    "is prompted by user": False,
    "is start perseverance": False,
    "last 5 eval scores": [],
    "last interaction datetime": None,
    "last update datetime": None,
    "feelings index": None,
    "next checkin datetime": None,
    "num of days since last eval": 0,
    "num of days since last prompt": 0,
    "num of days since last perseverance": 0,
    "num of days since last mindfulness": 0,
    "num of days since last goal setting": 0,
    "num of prompted today": 0,
    "perseverance counter": 0,
    "reading eval index": 0,
    "reading eval data": []
}


class VisionProjectDelegator:

    def __init__(
            self,
            statedb,
            update_window_seconds=5,
            scheduled_window_minutes=15,
            seconds_between_interactions=15,
            max_num_of_prompted_per_day=3
    ):
        self._state_database = statedb
        self._update_window_seconds = datetime.timedelta(seconds=update_window_seconds)
        self._scheduled_window_minutes = datetime.timedelta(minutes=scheduled_window_minutes)
        self._seconds_between_interactions = datetime.timedelta(seconds=seconds_between_interactions)
        self._max_num_of_prompted_per_day = max_num_of_prompted_per_day

        self._is_run_interaction = False

        # for testing purposes
        self._reset_database()

    def update(self):
        if not self._state_database.is_set("last update datetime"):
            self._state_database.set("last update datetime", datetime.datetime.now())
        if self._is_new_day():
            self._daily_state_update()
            self._update_act_variables()
            self._state_database.set("num of prompted today", 0)
            self._state_database.set("perseverance counter", 0)
            self._state_database.set("feelings index", None)
        self._state_database.set("last update datetime", datetime.datetime.now())

    def _is_new_day(self):
        return datetime.datetime.now().date() > self._state_database.get("last update datetime").date()

    def _daily_state_update(self):
        keys_to_check = {
            "is done eval today": "num of days since last eval",
            "is done prompted today": "num of days since last prompt",
            "is done perseverance today": "num of days since last perseverance",
            "is done mindfulness today": "num of days since last mindfulness",
            "is done goal setting today": "num of days since last goal setting"
        }
        for key in keys_to_check:
            if not self._state_database.get(key):
                old_value = self._state_database.get(keys_to_check[key])
                self._state_database.set(keys_to_check[key], old_value + 1)
            else:
                self._state_database.set(keys_to_check[key], 0)
            self._state_database.set(key, False)

    def _update_act_variables(self):
        last_5_scores = self._state_database.get("last 5 eval scores")
        if len(last_5_scores) == 5:
            last_5_scores.pop(0)
        last_5_scores.append(self._state_database.get("current eval score"))
        self._state_database.set("last 5 eval scores", last_5_scores)

    def get_interaction_type(self):
        # logging.info("Determining interaction type")
        # if self._is_first_interaction():
        #     interaction_type = Interactions.FIRST_INTERACTION
        # elif self._is_time_for_scheduled_interaction():
        #     interaction_type = Interactions.SCHEDULED_INTERACTION
        # elif self._is_run_too_many_prompted():
        #     interaction_type = Interactions.TOO_MANY_PROMPTED
        # elif self._is_run_prompted_interaction():
        #     interaction_type = Interactions.PROMPTED_INTERACTION
        # elif self._is_ask_to_do_evaluation():
        #     interaction_type = Interactions.ASK_TO_DO_EVALUATION
        # else:
        #     interaction_type = None
        interaction_type = Interactions.ASK_TO_DO_EVALUATION
        return interaction_type

    def _is_first_interaction(self):
        return not self._state_database.is_set("first interaction datetime")

    def _is_time_for_scheduled_interaction(self):
        if not self._state_database.get("next checkin datetime"):
            return False

        if self._state_database.get("is done eval today"):
            is_time = False
        else:
            is_in_update_window = self._is_in_window(
                self._state_database.get("next checkin datetime"),
                self._update_window_seconds,
                datetime.datetime.now()
            )
            is_prompted_in_scheduled_window = self._is_in_window(
                self._state_database.get("next checkin datetime"),
                self._scheduled_window_minutes,
                datetime.datetime.now()
            )

            is_time = is_in_update_window or \
                      (is_prompted_in_scheduled_window and not self._state_database.get("is done eval today") and
                       self._state_database.get("is prompted by user"))

        return is_time

    def _is_ask_to_do_evaluation(self):  # this graph begins with 'ask to chat'
        is_in_scheduled_window = self._is_in_window(
            self._state_database.get("next checkin datetime"),
            self._scheduled_window_minutes,
            datetime.datetime.now()
        )
        return self._state_database.get("is prompted by user") and \
            not self._state_database.get("is done eval today") and \
            not is_in_scheduled_window

    def _is_run_prompted_interaction(self):
        return self._state_database.get("is prompted by user") and \
            self._state_database.get("is done eval today") and \
            self._state_database.get("num of prompted today") < self._max_num_of_prompted_per_day

    def _is_run_too_many_prompted(self):
        return self._state_database.get("is prompted by user") and \
               self._state_database.get("num of prompted today") >= self._max_num_of_prompted_per_day

    def _is_in_window(self, center, window, time_to_check):
        start_time = center - window
        end_time = center + window
        return start_time <= time_to_check <= end_time

    def _reset_database(self):
        for key in INITIAL_STATE_DB:
            self._state_database.set(key, INITIAL_STATE_DB[key])
