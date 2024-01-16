#!/usr/bin/env python3
# import rospy
# from std_msgs.msg import String
# from sensor_msgs.msg import Mouse

import warnings
import logging 
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity

class Invoker():
    """
    Invoker class in the Command Pattern.

    The Invoker is responsible for managing and executing commands. It acts as an intermediary
    between the client and the actual command objects. It encapsulates the execution of commands,
    allowing for parameterization, queuing, and logging.

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
    logger = logging.getLogger("Invoker")
    __instance = None

    def __init__(self, game) -> None:
        """
        Initialize the Invoker with an empty list to store commands.
        """
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)
        self.__has_attack_player = False
        self.__has_defend_player = False

        self.__client_A_player = None
        self.__client_B_player = None
        self.__server_player = None
        self.__game = game
        # self.client_A_subsriber = rospy.Subscriber('/client_A', String, self.client_A_callback)
        # self.client_B_subsriber = rospy.Subscriber('/client_B', String, self.client_B_callback)

    @classmethod
    def get_instance():
        if Invoker.__instance == None:
            Invoker.__instance = Invoker()
        
        return Invoker.__instance
    
    def get_client_A_player(self):
        return self.__client_A_player
    
    def get_client_B_player(self):
        return self.__client_B_player

    def get_server_player(self):
        return self.__server_player

    def run_terminal(self):
        # while not rospy.is_shutdown():
        while True:
            input_str = input()
            Invoker.logger.info(f"Received Command \"{input_str}\" from terminal")

            input_str_list = input_str.split()

            # signin A dctime
            if (input_str_list[0] == "signin"):
                if (self.__server_player == None):
                    if (len(input_str_list) == 3):
                        if (input_str_list[1] == "A" and self.__has_attack_player == False):
                            self.__server_player = Player(input_str_list[2], Identity.ATTACK, self.__game.get_commander())
                            self.__has_attack_player = True
                            Invoker.logger.info(f"Server signed in! Identity: Attack")
                        elif (input_str_list[1] == "D" and self.__has_defend_player == False):
                            self.__server_player = Player(input_str_list[2], Identity.DEFEND, self.__game.get_commander())
                            self.__has_defend_player = True
                            Invoker.logger.info(f"Server signed in! Identity: Defend")
                        else:
                            Invoker.logger.error("Type must be capital A (Attacker) or D (Defender) or someone already get the role")
                    else:
                        Invoker.logger.error("signin args must be 2. Example command \"signin A DCtime\"")
                else:
                    Invoker.logger.error("Server already signin! Use signout to change your identity")

            # signout  
            elif (input_str_list[0] == "signout"):
                if (len(input_str_list) == 1 and self.__server_player != None):
                    if (self.__server_player.get_identity() == Identity.ATTACK):
                        self.__has_attack_player = False
                    elif (self.__server_player.get_identity() == Identity.DEFEND):
                        self.__has_defend_player = False

                    self.__server_player = None
                    Invoker.logger.info("Server signed out")
                elif (self.__server_player == None):
                    Invoker.logger.error("You can only signout after signin.")
                else:
                    Invoker.logger.error("signout args must be 0")

            # status
            elif (input_str_list[0] == "status"):
                if (len(input_str_list) == 1):
                    Invoker.logger.info("--- Server Status ---")
                    Invoker.logger.info(f"ClientA:{self.__client_A_player}")
                    Invoker.logger.info(f"ClientB:{self.__client_B_player}")
                    Invoker.logger.info(f"Server:{self.__server_player}")
                    Invoker.logger.info(f"has_attack_player:{self.__has_attack_player}")
                    Invoker.logger.info(f"has_defend_player:{self.__has_defend_player}")
                    Invoker.logger.info(f"--------------------")
                else:
                    Invoker.logger.error("status args must be 0")

            else:
                if (self.__server_player == None):
                    Invoker.logger.error("You havent sign in yet. Signin to affect the game")
                else:
                    self.__server_player.execute_command(input_str)


            

    # def client_A_callback(self, message):
    #     Invoker.logger.info(message)
    #     Invoker.logger.info(type(message))

    # def client_B_callback(self, message):
    #     Invoker.logger.info(message)
    #     Invoker.logger.info(type(message))
