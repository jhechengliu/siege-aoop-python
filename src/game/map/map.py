#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import UInt8MultiArray
# from sensor_msgs.msg import Mouse
from game.map_data_processor import MapDataProcessor
from game.game_data_publisher import GameDataPublisher
from game.game_flow_director import GameFlowDirector
from collections import deque
from game.pawn.attaker import Attacker
from game.pawn.defender import Defender
from game.player import Player
import json
import warnings

class Map:
    """
    Map is where the battle situated

    Attributes:
        __map (dict): key is (x, y) and value is map object
        __defend_player (Player): The real player who is the defending side
        __attack_player (Player): The real player who is the attacking side
        __defenders (deque):  contains operators who is defending the map
        __attackers (deque):  contains operators who is attacking the map
        __map_data_processor: an object which process the map into two maps for two players
        __game_data_publisher: an object which get the data and sent the data to the client unity side
        __game_flow_director: an object which is the main judge of the game
    """

    # singleton instance sits here
    __instance = None
    def __init__(self, map:dict, defend_player:Player, attack_player:Player, defender_count:int, attacker_count:int):
        """
        Don't use constructor to init this class, use get_instance method instead

        Attributes:
            map (dict): a dict that its keys are location and values are map objects
            defend_player (Player): The real player who is the defend team
            attack_player (Player): The real player who is the attacking side
            defender_count (int): The count of the operators who is defending the map. Default is 5 operators
            attacker_count (int): The count of the operators who is attacking the map. Default is 5 operators

        Returns:
            Dont use this thing you XXXXXX
        """
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)
        self.__map = map
        self.__defend_player = defend_player
        self.__attack_player = attack_player
        self.__defenders = deque()
        self.__attackers = deque()
        self.__map_data_processor = MapDataProcessor.get_instance()
        self.__game_data_publisher = GameDataPublisher.get_instance()
        self.__game_flow_director = GameFlowDirector.get_instance()

        for _ in range(defender_count):
            self.__defenders.append(Defender(self.__defend_player))

        for _ in range(attacker_count):
            self.__attackers.append(Attacker(self.__attack_player))

    @classmethod
    def get_instance(cls, map:dict, defend_player:Player, attack_player:Player, defender_count=5, attacker_count=5):
        """
        Use this method to get the instance of the only Map running in the program

        Atributes (Only matters when this is the first time init the class):
            map (dict): a dict that its keys are location and values are map objects
            defend_player (Player): The real player who is the defend team
            attack_player (Player): The real player who is the attacking side
            defender_count (int): The count of the operators who is defending the map. Default is 5 operators
            attacker_count (int): The count of the operators who is attacking the map. Default is 5 operators

        Returns:
            A Map object
        """
        if Map.__instance == None:
            warnings.warn("Use get_instance class method to obtain the instance", UserWarning)
            Map.__instance = Map(map, defend_player, attack_player, defender_count, attacker_count)

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