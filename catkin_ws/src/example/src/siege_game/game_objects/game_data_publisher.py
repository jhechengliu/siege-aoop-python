#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json

from siege_game.game_objects.logger import Logger

class ServerDetectClientAListener(rospy.SubscribeListener):
    logger = Logger("ServerDetectClientAListener")
    def __init__(self, __game_data_publisher):
        self.__game_data_publisher:GameDataPublisher = __game_data_publisher

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        ServerDetectClientAListener.logger.info("A peer subscribed to topic [%s]"%topic_name)
        peer_publish(String(str))
        
    def peer_unsubscribe(self, topic_name, numPeers):
        ServerDetectClientAListener.logger.info("A peer unsubscribed from topic [%s]"%topic_name)
        ServerDetectClientAListener.logger.fatal("BOOOM")
        self.__game_data_publisher.boomClientA()
        self.__game_data_publisher.boomClientB()
        

class GameDataPublisher:
    logger = Logger('GameDataPublisher')

    def __init__(self, game_id:str):
        GameDataPublisher.logger.warning("<game_data_publisher.py> Use get_instance class method to obtain the instance")
        self.__server_signin_publisher = rospy.Publisher('/server_signin', String, queue_size=10)
        self.__server_signin_message = String()

        self.__server_detect_client_A = rospy.Publisher('/server_detect_client_A', String, queue_size=50, subscriber_listener=ServerDetectClientAListener(self))
        self.__server_detect_client_A_data = String()
        self.__server_detect_client_A_data.data = "safe"

        self.__server_detect_client_B = rospy.Publisher('/server_detect_client_B', String, queue_size=50, subscriber_listener=ServerDetectClientAListener(self))
        self.__server_detect_client_B_data = String()
        self.__server_detect_client_B_data.data = "safe"

        self.__server_client_A_publisher = rospy.Publisher('/server_client_A_' + game_id, String, queue_size=10)
        self.__server_client_B_publisher = rospy.Publisher('/server_client_B_' + game_id, String, queue_size=10)
        self.__server_client_A_message = String()
        self.__server_client_B_message = String()
    
    def publish_server_signin(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_signin_message.data = full_msg
        GameDataPublisher.logger.info(f"Sending data to server signin channel: {full_msg}")
        self.__server_signin_publisher.publish(self.__server_signin_message)

    def boomClientA(self):
        self.__server_detect_client_A_data.data = "boom"
        GameDataPublisher.logger.fatal("Client A BOOM")

    def publishDetectClientA(self):
        print(self.__server_detect_client_A_data)
        print(self.__server_detect_client_A_data.data)
        self.__server_detect_client_A.publish(self.__server_detect_client_A_data)

    def boomClientB(self):
        self.__server_detect_client_B_data.data = "boom"
        GameDataPublisher.logger.fatal("CLient B BOOM")

    def publishDetectClientB(self):
        self.__server_detect_client_B.publish(self.__server_detect_client_B_data)

    def publish_client_A_server_actively(self, msg):
        self.__server_client_A_message.data = msg
        GameDataPublisher.logger.info(f"Sending active data to server client A channel: {msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)

    def publish_client_B_server_actively(self, msg):
        self.__server_client_B_message.data = msg
        GameDataPublisher.logger.info(f"Sending active data to server client B channel: {msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)



