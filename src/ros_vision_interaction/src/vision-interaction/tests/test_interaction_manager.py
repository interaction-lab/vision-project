#!/usr/bin/python3.8
import datetime
import freezegun
import pytest

from controllers import InteractionManager
from vision_project_tools.vision_engine import VisionInteractionEngine as InteractionEngine


def mock_run(_, planner):

    while planner.is_active:
        while planner.is_active:
            for node_name in mock_run_next_plan(planner):
                yield node_name


def mock_run_next_plan(planner):
    message_name, pre_hook, post_hook = planner.pop_plan(is_return_hooks=True)
    yield message_name

    pre_hook()
    post_hook()


@pytest.fixture
def interaction_manager(statedb, interaction_builder, monkeypatch):
    monkeypatch.setattr(InteractionEngine, "modified_run", mock_run)
    manager = InteractionManager(
        statedb=statedb,
        interaction_builder=interaction_builder
    )
    return manager


def test_run_first_interaction(interaction_manager, statedb):
    assert not interaction_manager._state_database.is_set("first interaction")
    for _ in interaction_manager.run_interaction_once("first interaction"):
        pass
    assert interaction_manager._state_database.is_set("first interaction datetime")


def test_run_scheduled_interaction(interaction_manager, statedb):
    with freezegun.freeze_time("2021-02-10"):
        assert not statedb.is_set("last interaction time")
        for _ in interaction_manager.run_interaction_once("scheduled interaction"):
            pass
        assert statedb.is_set("is done eval today")


def test_run_reading_evaluation(interaction_manager, statedb):
    current_eval_index = statedb.get("reading eval index")
    for _ in interaction_manager.run_interaction_once("scheduled interaction"):
        pass
    assert statedb.is_set("is done eval today")
    assert statedb.get("is interaction finished")
    assert statedb.get("reading eval index") == current_eval_index + 1


def test_run_prompted_interaction(interaction_manager, statedb):
    with freezegun.freeze_time("2021-02-10"):
        assert statedb.get("num of prompted today") == 0
        for _ in interaction_manager.run_interaction_once("prompted interaction"):
            pass
        assert statedb.get("num of prompted today") == 1


def test_determine_is_do_goal_setting(interaction_manager, statedb):
    # less than a week after first interaction
    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set("first interaction datetime", first_interaction_datetime)
    statedb.set("feelings index", 2)
    statedb.set("num of days since last prompt", 3)
    statedb.set("num of days since last perseverance", 3)
    with freezegun.freeze_time("2021-04-05"):
        assert not interaction_manager._is_do_goal_setting()

    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set("first interaction datetime", first_interaction_datetime)
    statedb.set("feelings index", 2)
    statedb.set("num of days since last prompt", 3)
    statedb.set("num of days since last perseverance", 3)
    statedb.set("num of days since last goal setting", 6)
    statedb.set("last 5 eval scores", [5, 5, 5, 5, 5])
    statedb.set("current eval score", 4)
    with freezegun.freeze_time("2021-04-10"):
        assert not interaction_manager._is_do_goal_setting()
        statedb.set("num of days since last goal setting", 7)
        assert interaction_manager._is_do_goal_setting()


def test_determine_is_do_mindfulness(interaction_manager, statedb):
    # less than a week after first interaction
    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set("first interaction datetime", first_interaction_datetime)
    with freezegun.freeze_time("2021-04-05"):
        assert not interaction_manager._is_do_mindfulness()

    first_interaction_datetime = datetime.datetime(2021, 4, 1, 12, 0, 0)
    statedb.set("first interaction datetime", first_interaction_datetime)
    statedb.set("feelings index", 2)
    statedb.set("num of days since last mindfulness", 2)
    statedb.set("last 5 eval scores", [5, 5, 5, 5, 5])
    statedb.set("current eval score", 4)
    with freezegun.freeze_time("2021-04-10"):
        assert interaction_manager._is_do_mindfulness()
