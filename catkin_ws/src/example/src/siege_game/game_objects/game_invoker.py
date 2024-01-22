#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
# from sensor_msgs.msg import Mouse

import warnings
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from typing import List, Callable
from siege_game.game import Game

class GameInvoker():

    def __init__(self, game_id):

        self.__client_A_player:Player = None
        self.__client_B_player:Player = None
        self.__client_A_connected = False
        self.__client_B_connected = False

        self.__game_id = game_id
        self.__logger = Logger("GameInvoker_" + self.__game_id)

        self.__client_A_subsriber = rospy.Subscriber('/client_A_' + self.__game_id, String, self.client_A_callback)
        self.__client_B_subsriber = rospy.Subscriber('/client_B_' + self.__game_id, String, self.client_B_callback)

        self.__server_client_A_publisher = rospy.Publisher('/server_client_A_' + self.__game_id, String, queue_size=10)
        self.__server_client_B_publisher = rospy.Publisher('/server_client_B_' + self.__game_id, String, queue_size=10)

        self.__server_client_A_message = String()
        self.__server_client_B_message = String()

        self.__game = Game(game_id, self)

    def print_status(self):
        # self.__logger.debug(f"----- ID: {self.__game_id} Status -----")
        # self.__logger.debug(f"Client A Player Object:{self.__client_A_player}")
        # self.__logger.debug(f"Client B Player Object:{self.__client_B_player}")
        # self.__logger.debug(f"Client A Connected: {self.__client_A_connected}")
        # self.__logger.debug(f"Client B Connected: {self.__client_B_connected}")
        # self.__logger.debug(f"-------------------------------------------")
        pass

    def get_client_A_player(self):
        return self.__client_A_player
    
    def get_client_B_player(self):
        return self.__client_B_player
    
    def get_client_A_connected(self):
        return self.__client_A_connected
    
    def get_client_B_connected(self):
        return self.__client_B_connected
    
    def set_client_A_connected(self, connected:bool):
        self.__client_A_connected = connected

    def set_client_B_connected(self, connected:bool):
        self.__client_B_connected = connected
    
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
            self.__logger.error(f"Client A received a blank message! Hmm")
            return
        
        elif len(message_str_list) == 1:
            self.__logger.error(f"id: {message_str_list[0]} send a empty command")
            self.__logger.debug(f"Returns: \"empty_command_error\" back to client")
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
            self.__sign_in_process(id, args, 'A', self.publish_client_A_server, self.publish_client_B_server_actively, self.__client_A_player)

        elif heading == "signout":
            self.__sign_out_process(id, args, 'A', self.publish_client_A_server, self.__client_A_player)

        else:
            self.__game_invoker_client_A_player_execute_command(id, command)

    def client_B_callback(self, message):
        message_str_list = message.data.split()
        command = " ".join(message_str_list[1:])    #connect back, space between; exclude id
        id = None
        heading = None
        args = None

        if len(message_str_list) == 0:
            self.__logger.error(f"Client B received a blank message! Hmm")
            return
        
        elif len(message_str_list) == 1:
            self.__logger.error(f"id: {message_str_list[0]} send a empty command")
            self.publish_server_connect(message_str_list[0], "empty_command_error")
            self.__logger.debug(f"Returns: \"empty_command_error\" back to client")
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
            self.__sign_in_process(id, args, 'B', self.publish_client_B_server, self.publish_client_A_server_actively, self.__client_B_player)
        elif heading == "signout":
            self.__sign_out_process(id, args, 'B', self.publish_client_B_server, self.__client_B_player)

        else:
            # after signed in, use player to call commander and execute command
            self.__game_invoker_client_B_player_execute_command(id, command)
            
    def __client_get_message_info(self, A_or_B:str, id, heading, args):
        # self.__logger.info(f"client {A_or_B} callback received message:")
        # self.__logger.info("================")
        # self.__logger.info(f"id: {id}")
        # self.__logger.info(f"heading: {heading}")
        # self.__logger.info(f"args: {args}")
        # self.__logger.info("================")
        pass

    def make_client_A_player(self, name, identity, commander):
        self.__client_A_player = Player(name, identity, commander)

    def make_client_B_player(self, name, identity, commander):
        self.__client_B_player = Player(name, identity, commander)

    def __sign_in_process(self, id, args:List, A_or_B:str, publish_function:Callable, opponent_active_publish_function:Callable, client_player):
        if len(args) == 2:
                if (client_player != None):
                    self.__logger.error(f"Client {A_or_B} already signed in")
                    publish_function(id, "already_signin")
                    return
                
                elif (client_player == None):
                    identity = None

                    if (args[0] == 'A'):
                        if (A_or_B == 'A' and self.__client_B_player != None and self.__client_B_player.get_identity() == Identity.ATTACK):
                            self.__logger.error("Attacker is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        elif (A_or_B == 'B' and self.__client_A_player != None and self.__client_A_player.get_identity() == Identity.ATTACK):
                            self.__logger.error("Attacker is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        identity = Identity.ATTACK
                    elif (args[0] == 'D'):
                        if (A_or_B == 'A' and self.__client_B_player != None and self.__client_B_player.get_identity() == Identity.DEFEND):
                            self.__logger.error("Defender is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        if (A_or_B == 'B' and self.__client_A_player != None and self.__client_A_player.get_identity() == Identity.DEFEND):
                            self.__logger.error("Defender is occupied")
                            publish_function(id, "identity_used")
                            return
                        
                        identity = Identity.DEFEND
                    else:
                        self.__logger.error("Identity must be 'A' (Attacker) or 'D' (Defender)")
                        publish_function(id, "identity_error")
                        return

                    if (A_or_B == 'A'):
                        self.make_client_A_player(args[1], identity, self.__game.get_commander())
                    elif (A_or_B == 'B'):
                        self.make_client_B_player(args[1], identity, self.__game.get_commander())

                    

                    opponent_identity = "none"
                    opponent_name = "none"
                    get_str_identity = lambda identity, name : f"occupied_attacker_{name}" if identity == Identity.ATTACK else (f"occupied_defender_{name}" if identity == Identity.DEFEND else f"occupied_none_{name}")
                    if (A_or_B == 'A'):
                        self.__logger.info(f"Checking Opponent Client B player name and identity: {self.__client_B_player}")
                        if (self.__client_B_player == None):
                            self.__logger.info("Client B isnt ready yet")
                            publish_function(id, f"success_{opponent_name}_{opponent_identity}")
                            opponent_active_publish_function(get_str_identity(identity, args[1]))
                        else:
                            self.__logger.info("Client B is ready")
                            if (self.__client_B_player.get_identity() == Identity.ATTACK):
                                opponent_identity = "attacker"
                            elif (self.__client_B_player.get_identity() == Identity.DEFEND):
                                opponent_identity = "defender"
                            opponent_name = self.__client_B_player.get_name()

                            publish_function(id, f"success_{opponent_name}_{opponent_identity}")
                            opponent_active_publish_function(get_str_identity(identity, args[1]))
                        self.__logger.debug(f"Success! Client {A_or_B} Player: {self.__client_A_player}")
                        self.__check_players_and_send_start_setting()

                    elif (A_or_B == 'B'):
                        self.__logger.info(f"Checking Opponent Client A player name and identity {self.__client_A_player}")
                        if (self.__client_A_player == None):
                            self.__logger.info("Client A isnt ready yet")
                            publish_function(id, f"success_{opponent_name}_{opponent_identity}")
                            opponent_active_publish_function(get_str_identity(identity, args[1]))
                        else:
                            self.__logger.info("Client A is ready")
                            if (self.__client_A_player.get_identity() == Identity.ATTACK):
                                opponent_identity = "attacker"
                            elif (self.__client_A_player.get_identity() == Identity.DEFEND):
                                opponent_identity = "defender"
                            opponent_name = self.__client_A_player.get_name()

                            publish_function(id, f"success_{opponent_name}_{opponent_identity}")
                            opponent_active_publish_function(get_str_identity(identity, args[1]))
                        self.__logger.debug(f"Success! Client {A_or_B} Player: {self.__client_B_player}")
                        self.__check_players_and_send_start_setting()

          
        else:
            self.__logger.error("signin commands args not equal to 2")
            publish_function(id, "args_len_error")

    def __sign_out_process(self, id, args:List, A_or_B:str, publish_function:Callable, client_player):
        if (len(args) != 0):
            self.__logger.error("signout Args len must be 0")
            publish_function(id, "args_len_error")
            return

        elif (len(args) == 0):
            if (client_player == None):
                self.__logger.error("already signed out")
                publish_function(id, "already_signout")
                return
            
            elif (client_player != None):
                client_player = None
                self.__logger.debug(f"Client {A_or_B} signed out")
                publish_function(id, "success")
                return
            
    def publish_client_A_server(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_client_A_message.data = full_msg
        self.__logger.info(f"Sending data to server client A channel: {full_msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)
    
    def publish_client_A_server_actively(self, msg):
        self.__server_client_A_message.data = msg
        self.__logger.info(f"Sending active data to server client A channel: {msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)

    def publish_client_B_server(self, id, msg):
        full_msg = f"{id} {msg}"
        self.__server_client_B_message.data = full_msg
        self.__logger.info(f"Sending data to server client B channel: {full_msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)

    def publish_client_B_server_actively(self, msg):
        self.__server_client_B_message.data = msg
        self.__logger.info(f"Sending active data to server client B channel: {msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)

    def __game_invoker_client_A_player_execute_command(self, id, command):
        if self.__client_A_player == None:
            self.__logger.error("Client A need to sign in in order to use other commands to affect the game")
            self.publish_client_A_server(id, "not_signed_in_error")
            self.__logger.debug(f"Returns: \"not_signed_in_error\" back to client")
        else:
            reply = self.__client_A_player.execute_command(command)
            self.__logger.debug(f"Returns: \"{reply}\" back to client")
            self.publish_client_A_server(id, reply)

    def __game_invoker_client_B_player_execute_command(self, id, command):
        if self.__client_B_player == None:
            self.__logger.error("Client B need to sign in in order to use other commands to affect the game")
            self.publish_client_B_server(id, "not_signed_in_error")
            self.__logger.debug(f"Returns: \"not_signed_in_error\" back to client")
        else:
            reply = self.__client_B_player.execute_command(command)
            self.__logger.debug(f"Returns: \"{reply}\" back to client")
            self.publish_client_B_server(id, reply)

    def __check_players_and_send_start_setting(self):
        self.__logger.debug(f"checking players: A: {self.__client_A_player}, B: {self.__client_B_player}")
        self.__logger.debug(f"Pass the Test?: {self.__client_A_player and self.__client_B_player}")
        if self.__client_A_player and self.__client_B_player:
            rospy.sleep(3)
            self.publish_client_A_server_actively("start_setting")
            self.publish_client_B_server_actively("start_setting")
            self.__game.get_map().get_game_flow_director().next_state()
