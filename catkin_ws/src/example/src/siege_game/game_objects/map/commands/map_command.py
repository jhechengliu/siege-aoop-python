from abc import ABC
import abc
from siege_game.game_objects.states.state import State
from siege_game.game_objects.map.map import Map
from siege_game.game_objects.constants.identity import Identity
from typing import Tuple

import typing

class MapCommand(ABC):
    def __init__(self, game, args:Tuple[str], player):
        from siege_game.game import Game
        self.__game:Game = game
        self.__args:Tuple[str] = args
        self.__send_player = player
        self.__identity:Identity = player.get_identity()
        self.ready_count = 0

    @abc.abstractmethod
    def execute(self) -> None:
        raise NotImplementedError()
    
    @abc.abstractmethod
    def check(self) -> str: 
        raise NotImplementedError()
    
    def get_map(self):
        return self.__game.get_map()
    
    def get_args(self) -> Tuple:
        return self.__args
    
    def get_identity(self):
        return self.__identity
    
    def get_send_player(self):
        return self.__send_player
    
    def get_game(self):
        return self.__game
    

    

    
    
    