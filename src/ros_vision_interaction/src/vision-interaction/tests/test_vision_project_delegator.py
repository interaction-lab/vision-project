#!/usr/bin/python3.8
import datetime
import freezegun
import pytest

from controllers import VisionProjectDelegator


@pytest.fixture
def vision_project_delegator(statedb):
    # TODO: make params env variables
    return VisionProjectDelegator(
        statedb,
        update_window_seconds=15,
        seconds_between_interactions=1
    )


def check_database_values(statedb, **kwargs):
    for key, value in kwargs.items():
        assert statedb.get(key) == value


def test_determine_first_interaction(vision_project_delegator, statedb):
    assert vision_project_delegator._is_first_interaction()
    assert not vision_project_delegator._is_time_for_scheduled_interaction()
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 15, 6, 0, 0))
    assert not vision_project_delegator._is_first_interaction()


def test_determine_scheduled_interaction_not_prompted(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("is prompted by user", False)
    statedb.set("is done eval today", False)
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)

    scheduled_window_times = [
        "2021-1-12 11:59:46",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:14",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:15",
    ]
    # check that scheduled is called when in time window
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator._is_time_for_scheduled_interaction()
            assert vision_project_delegator.get_interaction_type() == "scheduled interaction"

    outside_scheduled_window_times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 12:00:16"
    ]
    # check scheduled is not called when not in time window
    for invalid_time in outside_scheduled_window_times:
        with freezegun.freeze_time(invalid_time):
            assert not vision_project_delegator._is_time_for_scheduled_interaction()


def test_determine_interaction_prompted_no_evaluation(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    statedb.set("is prompted by user", True)
    statedb.set("is done eval today", False)
    # check scheduled interaction is called when within scheduled interaction window and evaluation not done yet
    # window of +/- 15 minutes
    scheduled_window_times = [
        "2021-1-12 11:45:00",
        "2021-1-12 11:46:00",
        "2021-1-12 12:00:00",
        "2021-1-12 12:14:00",
        "2021-1-12 12:15:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == "scheduled interaction"
    # check 'ask to run prompted' called when not within scheduled interaction window and evaluation not done yet
    scheduled_window_times = [
        "2021-1-12 11:44:59",
        "2021-1-12 12:15:01",
        "2021-1-12 11:00:00",
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
    ]
    for valid_time in scheduled_window_times:
        with freezegun.freeze_time(valid_time):
            assert vision_project_delegator.get_interaction_type() == "ask to do evaluation"


def test_determine_interaction_prompted_evaluation_done(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    statedb.set("last interaction datetime", datetime.datetime(2021, 1, 10, 6, 0, 0))
    next_checkin_time = datetime.datetime(2021, 1, 12, 12, 0, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    statedb.set("is prompted by user", True)
    statedb.set("is done eval today", True)
    # check that prompted interaction is called when evaluation has been done and QT is prompted
    times = [
        "2021-1-12 2:00:00",
        "2021-1-12 6:30:00",
        "2021-1-12 11:59:44",
        "2021-1-12 11:59:45",
        "2021-1-12 12:00:00",
        "2021-1-12 12:00:15",
        "2021-1-12 12:00:16"
    ]
    for time in times:
        with freezegun.freeze_time(time):
            assert vision_project_delegator.get_interaction_type() == "prompted interaction"


def test_is_done_reading_evaluation(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 2, 9, 6, 0, 0))
    statedb.set("next checkin datetime", datetime.datetime(2021, 2, 11, 2, 0, 0))
    statedb.set("last update datetime", datetime.datetime(2021, 2, 10, 12, 0, 0))
    statedb.set("num of days since last eval", 0)

    # check scheduled interaction is called when valid time and evaluation not done yet
    with freezegun.freeze_time("2021-02-11 2:00:00"):
        assert statedb.get("last update datetime") == datetime.datetime(2021, 2, 10, 12, 0, 0)
        vision_project_delegator.update()
        assert not statedb.get("is done eval today")
        assert vision_project_delegator.get_interaction_type() == "scheduled interaction"

        # check no interaction is called when valid time but evaluation already done
        with freezegun.freeze_time("2021-02-11 2:00:00"):
            statedb.set("is done eval today", True)
            assert vision_project_delegator.get_interaction_type() is None


def test_too_many_prompts(vision_project_delegator, statedb):
    statedb.set("first interaction datetime", datetime.datetime(2021, 2, 9, 6, 0, 0))
    statedb.set("last update datetime", datetime.datetime(2021, 2, 10, 12, 0, 0))
    next_checkin_time = datetime.datetime(2021, 2, 10, 12, 0, 0)
    statedb.set("next checkin datetime", next_checkin_time)
    statedb.set("num of prompted today", 3)
    statedb.set("is prompted by user", True)
    statedb.set("is done eval today", True)
    assert vision_project_delegator.get_interaction_type() == "too many prompted"


def test_new_day_update(vision_project_delegator, statedb):
    test_periods = [
        [
            "2021-03-15 00:00:00",
            "2021-03-16 00:00:00",
            "2021-03-17 00:00:00",
            "2021-03-18 00:00:00",
            "2021-03-19 00:00:00",
            "2021-03-20 00:00:00",
            "2021-03-21 00:00:00",
        ],
        # month changes during test period
        [
            "2021-02-25 00:00:00",
            "2021-02-26 00:00:00",
            "2021-02-27 00:00:00",
            "2021-02-28 00:00:00",
            "2021-03-01 00:00:00",
            "2021-03-02 00:00:00",
            "2021-03-03 00:00:00",
        ],
        # year changes during test period
        [
            "2020-12-29 00:00:00",
            "2020-12-30 00:00:00",
            "2020-12-31 00:00:00",
            "2021-01-01 00:00:00",
            "2021-01-02 00:00:00",
            "2021-01-03 00:00:00",
            "2021-01-04 00:00:00",
        ]
    ]
    for period in test_periods:
        vision_project_delegator._reset_database()
        # day 1
        with freezegun.freeze_time(period[0]):
            vision_project_delegator.update()
            first_interaction_datetime = datetime.datetime(2021, 3, 15, 2, 0, 0, 0)
            statedb.set("first interaction datetime", first_interaction_datetime)
            statedb.set("last interaction datetime", first_interaction_datetime)
            statedb.set("is done eval today", True)
        # day 2
        with freezegun.freeze_time(period[1]):
            vision_project_delegator.update()
            expected_values = {
                "num of days since last eval": 0,
                "num of days since last prompt": 1,
                "num of days since last perseverance": 1,
                "num of days since last mindfulness": 1,
                "num of days since last goal setting": 1,
            }
            check_database_values(statedb, **expected_values)
        # day 3
        with freezegun.freeze_time(period[2]):
            vision_project_delegator.update()
            expected_values = {
                "is done eval today": False,
                "is done prompted today": False,
                "is done perseverance today": False,
                "is done mindfulness today": False,
                "is done goal setting today": False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                "num of days since last eval": 1,
                "num of days since last prompt": 2,
                "num of days since last perseverance": 2,
                "num of days since last mindfulness": 2,
                "num of days since last goal setting": 2,
            }
            check_database_values(statedb, **expected_values)
            statedb.set("is done eval today", True)
            statedb.set("is done prompted today", True)
            statedb.set("is done perseverance today", True)
            statedb.set("is done mindfulness today", True)
            statedb.set("is done goal setting today", True)
        # day 4
        with freezegun.freeze_time(period[3]):
            vision_project_delegator.update()
            expected_values = {
                "is done eval today": False,
                "is done prompted today": False,
                "is done perseverance today": False,
                "is done mindfulness today": False,
                "is done goal setting today": False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                "num of days since last eval": 0,
                "num of days since last prompt": 0,
                "num of days since last perseverance": 0,
                "num of days since last mindfulness": 0,
                "num of days since last goal setting": 0,
            }
            check_database_values(statedb, **expected_values)
            statedb.set("is done eval today", True)
            statedb.set("is done prompted today", True)
            statedb.set("is done perseverance today", True)
            statedb.set("is done mindfulness today", True)
            statedb.set("is done goal setting today", True)
        # day 5
        with freezegun.freeze_time(period[4]):
            vision_project_delegator.update()
            expected_values = {
                "is done eval today": False,
                "is done prompted today": False,
                "is done perseverance today": False,
                "is done mindfulness today": False,
                "is done goal setting today": False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                "num of days since last eval": 0,
                "num of days since last prompt": 0,
                "num of days since last perseverance": 0,
                "num of days since last mindfulness": 0,
                "num of days since last goal setting": 0,
            }
            check_database_values(statedb, **expected_values)
            statedb.set("is done eval today", False)
            statedb.set("is done prompted today", True)
            statedb.set("is done perseverance today", False)
            statedb.set("is done mindfulness today", True)
            statedb.set("is done goal setting today", True)
        # day 6
        with freezegun.freeze_time(period[5]):
            vision_project_delegator.update()
            expected_values = {
                "is done eval today": False,
                "is done prompted today": False,
                "is done perseverance today": False,
                "is done mindfulness today": False,
                "is done goal setting today": False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                "num of days since last eval": 1,
                "num of days since last prompt": 0,
                "num of days since last perseverance": 1,
                "num of days since last mindfulness": 0,
                "num of days since last goal setting": 0,
            }
            check_database_values(statedb, **expected_values)
            statedb.set("is done eval today", False)
            statedb.set("is done prompted today", True)
            statedb.set("is done perseverance today", False)
            statedb.set("is done mindfulness today", False)
            statedb.set("is done goal setting today", True)
        # day 7
        with freezegun.freeze_time(period[6]):
            vision_project_delegator.update()
            expected_values = {
                "is done eval today": False,
                "is done prompted today": False,
                "is done perseverance today": False,
                "is done mindfulness today": False,
                "is done goal setting today": False,
            }
            check_database_values(statedb, **expected_values)
            expected_values = {
                "num of days since last eval": 2,
                "num of days since last prompt": 0,
                "num of days since last perseverance": 2,
                "num of days since last mindfulness": 1,
                "num of days since last goal setting": 0,
            }
            check_database_values(statedb, **expected_values)
