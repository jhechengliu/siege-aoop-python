#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Mouse

import warnings
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from typing import List, Callable, Dict
from siege_game.game_objects.game_invoker import GameInvoker
import shortuuid

class Invoker():
    logger = Logger("Invoker")

    def __init__(self) -> None:
        """
        Initialize the Invoker with an empty list to store commands.
        """
        Invoker.logger.warning("Use get_instance class method to obtain the instance")

        self.__game_invokers = {}
        self.__unfull_game_id = shortuuid.uuid()
        self.get_new_unfull_game(self.__unfull_game_id)

        self.connect_subscriber = rospy.Subscriber('/connect', String, self.connect_callback)

        self.__server_connect_publisher = rospy.Publisher('/server_connect', String, queue_size=10)
        self.__server_connect_message = String()
        
        self.__force_close = False

    def get_new_unfull_game(self, game_id):
        self.__game_invokers[game_id] = GameInvoker(game_id)
        Invoker.logger.debug(f"New Game Invoker id: {game_id} added")

    def del_game(self, game_id):
        del self.__game_invokers[game_id]
        Invoker.logger.debug(f"del Game Invoker id: {game_id}")
        
    def always_run(self):
        while (not rospy.is_shutdown()) and (not self.__force_close):
            pass

    def run(self):
        while (not rospy.is_shutdown()) and (not self.__force_close):
            # self.__map.get_game_data_publisher().publishDetectClientA()
            # self.__map.get_game_data_publisher().publishDetectClientB()
            # rospy.sleep(1.)
            pass


    def run_terminal(self):
        while not rospy.is_shutdown() and (not self.__force_close):
            input_str = input()
            Invoker.logger.info(f"Received Command \"{input_str}\" from terminal")

            input_str_list = input_str.split()

            # status
            if (input_str_list[0] == "status"):
                keys_list = list(self.__game_invokers.keys())
                if (len(input_str_list) == 1):
                    Invoker.logger.debug("----- Server Status-----")
                    Invoker.logger.debug("Running Games:")
                    for index in range(len(keys_list)):
                        Invoker.logger.debug(f"Key Index: {index}, Key: {keys_list[index]}")
                    Invoker.logger.debug("Current unfull game:")
                    Invoker.logger.debug(f"Key: {self.__unfull_game_id}")
                    Invoker.logger.debug("-------------------------")
                elif (len(input_str_list) == 2):
                    if (input_str_list[1] in self.__game_invokers.keys()):
                        self.__game_invokers[input_str_list[1]].print_status()
                    else:
                        try:
                            int(input_str_list[1])
                        except ValueError:
                            Invoker.logger.error(f"Game ID: {input_str_list[1]} not found")
                        else:
                            if (int(input_str_list[1]) >= 0 and int(input_str_list[1]) < len(keys_list)):
                                self.__game_invokers[keys_list[int(input_str_list[1])]].print_status()
                            else:
                                Invoker.logger.error(f"Game Index: {int(input_str_list[1])} not found")

                        
                else:
                    Invoker.logger.error("status args must be 0")

            elif (input_str_list[0] == "reboot"):
                if (len(input_str_list) == 1):
                    # self.__game.get_map().get_game_data_publisher().boomClientA()
                    # self.__game.get_map().get_game_data_publisher().boomClientB()
                    # Invoker.logger.fatal(f"Self destruction is activated!")
                    self.__force_close = True
                else:
                    Invoker.logger.error("reboot args must be 0")

    def connect_callback(self, message):
        message_str:str = message.data
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
                unfull_game_invoker:GameInvoker = self.__game_invokers[self.__unfull_game_id]
                if (unfull_game_invoker.get_client_A_connected() == False):
                    unfull_game_invoker.set_client_A_connected(True)
                    return "client_A_" + self.__unfull_game_id
                
                elif (unfull_game_invoker.get_client_A_connected() == True and unfull_game_invoker.get_client_B_connected() == False):
                    unfull_game_invoker.set_client_B_connected(True)
                    return_str = "client_B_" + self.__unfull_game_id
                    self.__unfull_game_id = shortuuid.uuid()
                    self.get_new_unfull_game(self.__unfull_game_id)
                    return return_str
                
                elif (unfull_game_invoker.get_client_A_connected() == True and unfull_game_invoker.get_client_B_connected() == True):
                    self.__unfull_game_id = shortuuid.uuid()
                    self.get_new_unfull_game(self.__unfull_game_id)
                    return "full"
                
                else:
                    Invoker.logger.fatal("Something went wrong at checking clients are used or not")
                    return "fatal_error"
                
    def publish_server_connect(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_connect_message.data = full_msg
        Invoker.logger.info(f"Sending data to server connect channel: {full_msg}")
        self.__server_connect_publisher.publish(self.__server_connect_message)


        


