#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from siege_game.game_objects.logger import Logger

class GameDataPublisher:
    logger = Logger('GameDataPublisher')

    def __init__(self):
        GameDataPublisher.logger.warning("<game_data_publisher.py> Use get_instance class method to obtain the instance")
        self.__server_signin_publisher = rospy.Publisher('/server_signin', String, queue_size=10)
        self.__server_signin_message = String()
    
    def publish_server_signin(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_signin_message.data = full_msg
        GameDataPublisher.logger.info(f"Sending data to server signin channel: {full_msg}")
        self.__server_signin_publisher.publish(self.__server_signin_message)

