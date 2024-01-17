#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Mouse

import warnings
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from typing import List

class Invoker():
    """
    Invoker class in the Command Pattern.

    The Invoker is responsible for managing and executing commands. It acts as an intermediary
    between the client and the actual command objects. It encapsulates the execution of commands,
    allowing for parameterization, queuing, and logger.

    Attributes:
        __instance (Invoker): The instance of the Invoker class.
        __commands (list): A list to store the history of executed commands.
        last_command (Command): The last executed command.

    Methods:
        get_instance(): Returns the instance of the Invoker class.
        execute(command): Execute the given command.
        undo(): Undo the last executed command.
        batch_execute(commands): Execute a batch of commands simultaneously.
    """
    logger = Logger("Invoker")
    __instance = None

    def __init__(self, game) -> None:
        """
        Initialize the Invoker with an empty list to store commands.
        """
        Invoker.logger.warning("Use get_instance class method to obtain the instance")
        self.__client_A_player = None
        self.__client_B_player = None
        self.__client_A_connected = False
        self.__client_B_connected = False
        self.__game = game
        self.client_A_subsriber = rospy.Subscriber('/client_A', String, self.client_A_callback)
        self.client_B_subsriber = rospy.Subscriber('/client_B', String, self.client_B_callback)
        self.connect_subscriber = rospy.Subscriber('/connect', String, self.connect_callback)

        self.__server_connect_publisher = rospy.Publisher('/server_connect', String, queue_size=10)
        self.__server_connect_message = String()

    @classmethod
    def get_instance():
        if Invoker.__instance == None:
            Invoker.__instance = Invoker()
        
        return Invoker.__instance
    
    def get_client_A_player(self):
        return self.__client_A_player
    
    def get_client_B_player(self):
        return self.__client_B_player

    def run_terminal(self):
        while not rospy.is_shutdown():
            input_str = input()
            Invoker.logger.info(f"Received Command \"{input_str}\" from terminal")

            input_str_list = input_str.split()

            # status
            if (input_str_list[0] == "status"):
                if (len(input_str_list) == 1):
                    Invoker.logger.info("--- Server Status ---")
                    Invoker.logger.info(f"ClientA:{self.__client_A_player}")
                    Invoker.logger.info(f"ClientB:{self.__client_B_player}")
                    Invoker.logger.info(f"--------------------")
                else:
                    Invoker.logger.error("status args must be 0")

    def client_A_callback(self, message):
        message_str = message.data
        Invoker.logger.info(f"client A received: {message_str}")

    def client_B_callback(self, message):
        message_str = message.data
        Invoker.logger.info(f"client B received: {message_str}")

    def connect_callback(self, message):
        message_str = message.data
        Invoker.logger.info(f"connect callback received: {message_str}")
        message_str_list = message_str.split()
        
        if len(message_str_list) == 1:
            Invoker.logger.error(f"id: {message_str_list[0]} send a empty command")
            self.publish_server_connect(message_str_list[0], "empty_command_error")
        
        elif len(message_str_list) >= 2:
            id = message_str_list[0]
            heading = message_str_list[1]
            args = message_str_list[2:]
            self.publish_server_connect(id, self.connect_execute(heading, args))
            

    def connect_execute(self, heading:str, args:List) -> str:
        """
        connect command all response:
        args_must_be_0
        client_A
        client_B
        full
        fatal_error
        """
        if (heading == "connect"):
            if (len(args) != 0):
                Invoker.logger.error("connect command args must be 0")
                return "args_must_be_0"
            else:
                if (self.__client_A_connected == False):
                    self.__client_A_connected = True
                    return "client_A"
                
                elif (self.__client_A_connected == True and self.__client_B_connected == False):
                    self.__client_B_connected = True
                    return "client_B"
                
                elif (self.__client_A_connected == True and self.__client_B_connected == True):
                    return "full"
                
                else:
                    Invoker.logger.fatal("Something went wrong at checking clients are used or not")
                    return "fatal_error"
                
    def publish_server_connect(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_connect_message.data = full_msg
        Invoker.logger.info(f"Sending data to server signin channel: {full_msg}")
        self.__server_connect_publisher.publish(self.__server_connect_message)


