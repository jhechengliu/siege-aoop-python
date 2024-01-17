#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Mouse

import warnings
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from typing import List, Callable

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
        self.__client_A_player:Player = None
        self.__client_B_player:Player = None
        self.__client_A_connected = False
        self.__client_B_connected = False
        self.__game = game
        self.client_A_subsriber = rospy.Subscriber('/client_A', String, self.client_A_callback)
        self.client_B_subsriber = rospy.Subscriber('/client_B', String, self.client_B_callback)
        self.connect_subscriber = rospy.Subscriber('/connect', String, self.connect_callback)

        self.__server_client_A_publisher = rospy.Publisher('/server_client_A', String, queue_size=10)
        self.__server_client_B_publisher = rospy.Publisher('/server_client_B', String, queue_size=10)
        self.__server_connect_publisher = rospy.Publisher('/server_connect', String, queue_size=10)
        self.__server_connect_message = String()
        self.__server_client_A_message = String()
        self.__server_client_B_message = String()

        self.__listener = rospy.SubscribeListener()
        self.__listener.peer_unsubscribe = self.disconnect_callback

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
                    Invoker.logger.debug("----- Server Status -----")
                    Invoker.logger.debug(f"Client A Player Object:{self.__client_A_player}")
                    Invoker.logger.debug(f"Client B Player Object:{self.__client_B_player}")
                    Invoker.logger.debug(f"Client A Connected: {self.__client_A_connected}")
                    Invoker.logger.debug(f"Client B Connected: {self.__client_B_connected}")
                    Invoker.logger.debug(f"------------------------")
                else:
                    Invoker.logger.error("status args must be 0")

    def client_A_callback(self, message):
        """
        global error:
        empty_command_error
        not_signed_in_error

        command_not_found_error
        command_denied_error
        """
        message_str_list = message.data.split()
        command = " ".join(message_str_list[1:])
        id = None
        heading = None
        args = None
        

        if len(message_str_list) == 0:
            Invoker.logger.error(f"Client A received a blank message! Hmm")
            return
        
        elif len(message_str_list) == 1:
            Invoker.logger.error(f"id: {message_str_list[0]} send a empty command")
            Invoker.logger.debug(f"Returns: \"empty_command_error\" back to client")
            self.publish_server_connect(message_str_list[0], "empty_command_error")
            return
        
        elif len(message_str_list) == 2:
            id = message_str_list[0]
            heading = message_str_list[1]
            args = []

        elif len(message_str_list) >= 3:
            id = message_str_list[0]
            heading = message_str_list[1]
            args = message_str_list[2:]
        
        self.__client_get_message_info('A', id, heading, args)

        """
        signin A dctime:
        success
        args_len_error
        already_signin
        identity_error
        """

        if heading == "signin":
            self.__sign_in_process(args, 'A', self.publish_client_A_server, self.__client_A_player)

        elif heading == "signout":
            self.__sign_out_process(args, 'A', self)

        else:
            if self.__client_A_player == None:
                Invoker.logger.error("Client A need to sign in in order to use other commands to affect the game")
                self.publish_client_A_server(id, "not_signed_in_error")
                Invoker.logger.debug(f"Returns: \"not_signed_in_error\" back to client")
            else:
                reply = self.__client_A_player.execute_command(command)
                Invoker.logger.debug(f"Returns: \"{reply}\" back to client")
                self.publish_client_A_server(id, reply)

    def client_B_callback(self, message):
        message_str_list = message.data.split()
        command = " ".join(message_str_list[1:])
        id = None
        heading = None
        args = None

        if len(message_str_list) == 0:
            Invoker.logger.error(f"Client B received a blank message! Hmm")
            return
        
        elif len(message_str_list) == 1:
            Invoker.logger.error(f"id: {message_str_list[0]} send a empty command")
            self.publish_server_connect(message_str_list[0], "empty_command_error")
            Invoker.logger.debug(f"Returns: \"empty_command_error\" back to client")
            return
        
        elif len(message_str_list) == 2:
            id = message_str_list[0]
            heading = message_str_list[1]
            args = []

        elif len(message_str_list) >= 3:
            id = message_str_list[0]
            heading = message_str_list[1]
            args = message_str_list[2:]
        
        self.__client_get_message_info('B', id, heading, args)

        """
        signin A dctime:
        success
        args_len_error
        already_signin
        identity_error
        identity_used
        """
        """
        command: signout
        success
        already_signout
        args_len_error
        """

        if heading == "signin":
            self.__sign_in_process(id, args, 'B', self.publish_client_B_server, self.__client_B_player)
        elif heading == "signout":
            self.__sign_out_process(id, args, 'B', self.publish_client_B_server, self.__client_B_player)

        else:
            if self.__client_B_player == None:
                Invoker.logger.error("Client B need to sign in in order to use other commands to affect the game")
                self.publish_client_B_server(id, "not_signed_in_error")
                Invoker.logger.debug(f"Returns: \"not_signed_in_error\" back to client")
            else:
                reply = self.__client_B_player.execute_command(command)
                Invoker.logger.debug(f"Returns: \"{reply}\" back to client")
                self.publish_client_B_server(id, reply)

    def disconnect_callback(self, message):
        Invoker.logger.debug("Somebody closes the connection")
            
    def __client_get_message_info(self, A_or_B:str, id, heading, args):
        Invoker.logger.info(f"client {A_or_B} callback received message:")
        Invoker.logger.info("================")
        Invoker.logger.info(f"id: {id}")
        Invoker.logger.info(f"heading: {heading}")
        Invoker.logger.info(f"args: {args}")
        Invoker.logger.info("================")

    def __sign_in_process(self, id, args:List, A_or_B:str, publish_function:Callable, client_player):
        if len(args) == 2:
                if (client_player != None):
                    Invoker.logger.error(f"Client {A_or_B} already signed in")
                    publish_function(id, "already_signin")
                    return
                
                elif (client_player == None):
                    identity = None

                    if (args[0] == 'A'):
                        if (A_or_B == 'A' and self.__client_B_player != None and self.__client_B_player.get_identity() == Identity.ATTACK):
                            Invoker.logger.error("Attacker is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        elif (A_or_B == 'B' and self.__client_A_player != None and self.__client_A_player.get_identity() == Identity.ATTACK):
                            Invoker.logger.error("Attacker is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        identity = Identity.ATTACK
                    elif (args[0] == 'D'):
                        if (A_or_B == 'A' and self.__client_B_player != None and self.__client_B_player.get_identity() == Identity.DEFEND):
                            Invoker.logger.error("Defender is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        if (A_or_B == 'B' and self.__client_A_player != None and self.__client_A_player.get_identity() == Identity.DEFEND):
                            Invoker.logger.error("Defender is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        identity = Identity.DEFEND
                    else:
                        Invoker.logger.error("Identity must be 'A' (Attacker) or 'D' (Defender)")
                        publish_function(id, "identity_error")

                    client_player = Player(args[1], identity, self.__game.get_commander())
                    Invoker.logger.debug(f"Success! Client {A_or_B} Player: {client_player}")
                    publish_function(id, "success")

                    
        else:
            Invoker.logger.error("signin commands args not equal to 2")
            publish_function(id, "args_len_error")

    def __sign_out_process(self, id, args:List, A_or_B:str, publish_function:Callable, client_player):
        if (len(args) != 0):
            Invoker.logger.error("signout Args len must be 0")
            publish_function(id, "args_len_error")
            return

        elif (len(args) == 0):
            if (client_player == None):
                Invoker.logger.error("already signed out")
                publish_function(id, "already_signout")
                return
            
            elif (client_player != None):
                client_player = None
                Invoker.logger.debug(f"Client {A_or_B} signed out")
                publish_function(id, "success")
                return

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
        Invoker.logger.info(f"Sending data to server connect channel: {full_msg}")
        self.__server_connect_publisher.publish(self.__server_connect_message)

    def publish_client_A_server(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_client_A_message.data = full_msg
        Invoker.logger.info(f"Sending data to server client A channel: {full_msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)

    def publish_client_B_server(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_client_B_message.data = full_msg
        Invoker.logger.info(f"Sending data to server client B channel: {full_msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)
        


