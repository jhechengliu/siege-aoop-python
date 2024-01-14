from abc import ABC
import abc
from siege_game.game_objects.states.state import State
from siege_game.game_objects.map.map import Map
from siege_game.game_objects.player import Player
import typing

class MapCommand(ABC):
    def __init__(self, map:Map, args:tuple[str]):
        self.__map:Map = map
        self.__args:tuple[str] = args

    @abc.abstractmethod
    def execute(self) -> None:
        raise NotImplementedError()
    
    @abc.abstractmethod
    def check(self) -> bool: 
        raise NotImplementedError()
    
    def get_map(self) -> Map:
        return self.__map
    
    def get_sender(self) -> Player:
        return self.__sender
    
    def get_args(self) -> Player:
        return self.__args
    

    

    
    
    