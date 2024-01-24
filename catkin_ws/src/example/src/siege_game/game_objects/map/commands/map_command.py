from abc import ABC
import abc
from siege_game.game_objects.constants.identity import Identity
from typing import Tuple

import typing

class MapCommand(ABC):
    """
    The MapCommand class is an abstract base class for commands in the game.

    This class should be subclassed by specific command classes that implement the execute() and check() methods.

    Attributes:
        __game (Game): A Game object representing the current game.
        __args (Tuple[str]): The arguments for the command.
        __send_player: The player who sent the command.
        __identity (Identity): The identity of the player who sent the command.
        ready_count (int): The number of times the command has been executed.

    Methods:
        execute() -> None:
            Executes the command. This method should be implemented by subclasses.

        check() -> str:
            Checks if the command can be executed. This method should be implemented by subclasses.

        get_args() -> Tuple:
            Returns the arguments for the command.

        get_identity():
            Returns the identity of the player who sent the command.

        get_send_player():
            Returns the player who sent the command.

        get_game():
            Returns the current game.
    """
    def __init__(self, game, args:Tuple[str], player):
        from siege_game.game import Game
        self.__game:Game = game
        self.__args:Tuple[str] = args
        self.__send_player = player
        self.__identity:Identity = player.get_identity()
        self.ready_count = 0

    @abc.abstractmethod
    def execute(self) -> None:
        """
        Executes the command.

        This method should be implemented by subclasses.
        """
        raise NotImplementedError()
    
    @abc.abstractmethod
    def check(self) -> str:
        """
        Checks if the command can be executed.

        This method should be implemented by subclasses.

        Returns:
            str: A denial reason if the command cannot be executed, or None if the command can be executed.
        """
        raise NotImplementedError()
    
    def get_args(self) -> Tuple:
        """
        Returns the arguments for the command.

        Returns:
            Tuple: The arguments for the command.
        """
        return self.__args
    
    def get_identity(self):
        """
        Returns the identity of the player who sent the command.

        Returns:
            Identity: The identity of the player who sent the command.
        """
        return self.__identity
    
    def get_send_player(self):
        """
        Returns the player who sent the command.

        Returns:
            Player: The player who sent the command.
        """
        return self.__send_player
    
    def get_game(self):
        """
        Returns the current game.

        Returns:
            Game: The current game.
        """
        return self.__game
    

    

    
    
    