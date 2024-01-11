#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import UInt8MultiArray
# from sensor_msgs.msg import Mouse
from game.map_data_processor import MapDataProcessor
from game.game_data_publisher import GameDataPublisher
from game.game_flow_director import GameFlowDirector
from collections import deque
import json
import warnings

class Map:
    """
    Map is where the battle situated

    Attributes:
        __map (dict): key is (x, y) and value is map object
        __players_a (deque):  contains players in A team who is playing the game
        __players_b (deque):  contains players in B team who is playing the game
        __map_data_processor: an object which process the map into two maps for two players
        __game_data_publisher: an object which get the data and sent the data to the client unity side
        __game_flow_director: an object which is the main judge of the game
    """

    # singleton instance sits here
    __instance = None
    def __init__(self):
        self.__map = {}
        self.__defender = deque()
        self.__attacker = deque()
        self.__map_data_processor = MapDataProcessor.get_instance()
        self.__game_data_publisher = GameDataPublisher.get_instance()
        self.__game_flow_director = GameFlowDirector.get_instance()

    @classmethod
    def get_instance():
        if Map.__instance == None:
            warnings.warn("Use get_instance class method to obtain the instance", UserWarning)
            Map.__instance = Map()

        return Map.__instance


        # self.map_publisher = rospy.Publisher('/map_information', UInt8MultiArray, queue_size=1)
        # self.attacker_mouse_subsriber = rospy.Subscriber('/attacker_mouse_event', UInt8MultiArray, self.attacker_mouse_callback)
        # self.defender_mouse_subsriber = rospy.Subscriber('/defender_mouse_event', UInt8MultiArray, self.defender_mouse_callback)
        # self.map_message = UInt8MultiArray()



    # def attacker_mouse_callback(self, message):
    #     print("message")
    #     print(type(message))

    # def defender_mouse_callback(self, message):
    #     print("message")
    #     print(type(message))