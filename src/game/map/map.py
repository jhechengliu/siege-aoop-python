#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Mouse
from game.map_data_processor import MapDataProcessor
from game.game_data_publisher import GameDataPublisher
from game.game_flow_director import GameFlowDirector
import json


class Map:
    def __init__(self):
        self.map_array = [[]]
        self.players_a = []
        self.players_b = []
        self.map_data_processor = MapDataProcessor.get_instance()
        self.game_data_publisher = GameDataPublisher.get_instance()
        self.game_flow_director = GameFlowDirector.get_instance()


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