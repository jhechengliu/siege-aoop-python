#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt8MultiArray
from sensor_msgs.msg import Mouse
import json


class Map:
    def __init__(self):
        self.map_array = None
        self.map_publisher = rospy.Publisher('/cube_position', UInt8MultiArray, queue_size=1)
        self.attacker_mouse_subsricer = rospy.Subscriber('/keyboard_command', UInt8MultiArray, self.attacker_mouse_callback)
        self.defender_mouse_subsricer = rospy.Subscriber('/keyboard_command', UInt8MultiArray, self.defender_mouse_callback)
        self.map_message = UInt8MultiArray()

    def __load_map():
        with open('../Assets/Data/map_example.json', 'r') as json_file:
            # Load the JSON content into a Python dictionary
            map_array = json.load(json_file)

    def attacker_mouse_callback(self, message):
        print("message")
        print(type(message))

    def defender_mouse_callback(self, message):
        print("message")
        print(type(message))