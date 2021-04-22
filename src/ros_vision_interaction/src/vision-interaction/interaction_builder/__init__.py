import datetime

from interaction_engine.messager.directed_graph import DirectedGraph
from interaction_engine.messager.node import Node
from interaction_engine.text_populator import TextPopulator
from interaction_engine.text_populator import DatabasePopulator
from interaction_engine.text_populator import VarietyPopulator


class Graphs:

    ASK_TO_CHAT = "ask to chat"
    ASK_TO_DO_PERSEVERANCE = "ask to do perseverance"
    ASK_TO_DO_SCHEDULED = "ask to do scheduled"
    CHECK_READING_ID = "check reading id"
    EVALUATION = "evaluation"
    FIRST_CHECKIN = "first checkin"
    GOAL_SETTING = "goal setting"
    GOODBYE = "goodbye"
    GREETING = "greeting"
    MINDFULNESS = "mindfulness"
    PERSEVERANCE = "perseverance"
    PROMPTED_CHECKIN = "prompted checkin"
    REWARD = "reward"
    SCHEDULED_CHECKIN = "scheduled checkin"
    SCHEDULE_NEXT_CHECKIN = "schedule next checkin"
    TALK_ABOUT_VISION = "talk about vision"
    TOO_MANY_PROMPTED = "too many prompted"


class InteractionBuilder:

    def __init__(
            self,
            interaction_dict,
            variations_files,
            statedb
    ):
        self._interaction_dict = interaction_dict
        self._variations_files = variations_files

        self._statedb = statedb

        self._database_populator = DatabasePopulator(self._statedb)
        self._variety_populator = VarietyPopulator(self._variations_files)

        self._text_populator = TextPopulator(self._variety_populator, self._database_populator)
        self._interactions = {
            Graphs.ASK_TO_CHAT: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.ASK_TO_CHAT,
                self._text_populator,
            ),
            Graphs.ASK_TO_DO_PERSEVERANCE: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.ASK_TO_DO_PERSEVERANCE,
                self._text_populator,
            ),
            Graphs.ASK_TO_DO_SCHEDULED: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.ASK_TO_DO_SCHEDULED,
                self._text_populator,
            ),
            Graphs.CHECK_READING_ID: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.CHECK_READING_ID,
                self._text_populator,
            ),
            Graphs.EVALUATION: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.EVALUATION,
                self._text_populator,
            ),
            Graphs.FIRST_CHECKIN: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.FIRST_CHECKIN,
                self._text_populator,
            ),
            Graphs.GOAL_SETTING: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.GOAL_SETTING,
                self._text_populator,
            ),
            Graphs.GOODBYE: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.GOODBYE,
                self._text_populator,
            ),
            Graphs.GREETING: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.GREETING,
                self._text_populator,
            ),
            Graphs.MINDFULNESS: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.MINDFULNESS,
                self._text_populator,
            ),
            Graphs.PERSEVERANCE: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.PERSEVERANCE,
                self._text_populator,
            ),
            Graphs.PROMPTED_CHECKIN: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.PROMPTED_CHECKIN,
                self._text_populator,
            ),
            Graphs.REWARD: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.REWARD,
                self._text_populator,
            ),
            Graphs.SCHEDULED_CHECKIN: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.SCHEDULED_CHECKIN,
                self._text_populator,
            ),
            Graphs.SCHEDULE_NEXT_CHECKIN: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.SCHEDULE_NEXT_CHECKIN,
                self._text_populator,
            ),
            Graphs.TALK_ABOUT_VISION: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.TALK_ABOUT_VISION,
                self._text_populator,
            ),
            Graphs.TOO_MANY_PROMPTED: self.build_graph_from_dict(
                self._interaction_dict,
                Graphs.TOO_MANY_PROMPTED,
                self._text_populator,
            )
        }

        self._possible_graphs = [graph for graph in self._interactions.values()]

    def build_graph_from_dict(
            self,
            interactions_dict,
            graph_name,
            text_populator=None,
            speaking_rate=None
    ):
        graph_dict = interactions_dict[graph_name]
        start_node_name = graph_dict["start_node_name"]
        nodes = []

        valid_speaking_rates = [
            "x-slow",
            "slow",
            "medium",
            "fast",
            "x-fast"
        ]
        if speaking_rate is not None and speaking_rate not in valid_speaking_rates:
            raise ValueError("Not a valid speaking rate")

        all_nodes_in_graph = graph_dict["nodes"]
        for node_name in all_nodes_in_graph.keys():
            node_info = all_nodes_in_graph[node_name]
            optional_values = {
                "args": None,
                "result_convert_from_str_fn": str,
                "result_db_key": None,
                "is_append_result": False,
                "tests": None,
                "is_confirm": False,
                "error_message": "Please enter a valid input",
                "error_options": ("Okay", "Oops")
            }
            for value in optional_values.keys():
                if value not in node_info:
                    node_info[value] = optional_values[value]

            if speaking_rate is not None:
                node_info["content"] = "<prosody rate=\"{speaking_rate}\">".format(speaking_rate=speaking_rate) + \
                                       node_info["content"] + "</prosody> "

            if node_info["result_convert_from_str_fn"] == "next day checkin time":
                node_info["result_convert_from_str_fn"] = self.next_day_checkin_time_from_str

            if node_info["tests"] == "check reading id":
                node_info["tests"] = self.check_reading_id

            node = Node(
                name=node_name,
                transitions=node_info["transitions"],
                content=node_info["content"],
                options=node_info["options"],
                message_type=node_info["message_type"],
                args=node_info["args"],
                result_convert_from_str_fn=node_info["result_convert_from_str_fn"],
                result_db_key=node_info["result_db_key"],
                is_append_result=node_info["is_append_result"],
                tests=node_info["tests"],
                is_confirm=node_info["is_confirm"],
                error_message=node_info["error_message"],
                error_options=node_info["error_options"],
                text_populator=text_populator
            )
            nodes.append(node)

        return DirectedGraph(
            name=graph_name,
            nodes=nodes,
            start_node=start_node_name
        )

    def next_day_checkin_time_from_str(self, time_string):
        next_checkin_time = datetime.datetime.strptime(time_string, '%I:%M %p').time()
        current_datetime = datetime.datetime.now()
        current_day = current_datetime.day
        return current_datetime.replace(
            day=current_day + 1,
            hour=next_checkin_time.hour,
            minute=next_checkin_time.minute
        )

    def check_reading_id(self, reading_id):
        eval_index = self._statedb.get("reading eval index")
        try:
            expected_id = self._statedb.get("reading eval data")[eval_index]["id"]
            correct_id = reading_id == expected_id
        except IndexError:
            correct_id = True
        return correct_id

    @property
    def interactions(self):
        return self._interactions

    @property
    def possible_graphs(self):
        return self._possible_graphs
