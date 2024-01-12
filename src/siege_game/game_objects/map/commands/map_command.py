from abc import ABC
import abc
from siege_game.game_objects.states.state import State
from siege_game.game_objects.map.map import Map
from siege_game.game_objects.player import Player
from siege_game.game import Game
import typing

class MapCommand(ABC):
    def __init__(self, sender:Player):
        self.__sender:Player = sender
        self.__map:Map = Game.get_instance()

    @abc.abstractmethod
    def execute(self) -> bool:
        if self.check():
            # code
            return True
        else:
            return False
    
    @abc.abstractmethod
    def check(self) -> bool: 
        raise NotImplementedError()
    
    @abc.abstractmethod
    def content(self) -> None:
        raise NotImplementedError()
    

    

    
    
    