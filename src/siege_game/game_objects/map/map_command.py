from abc import ABC
import abc
from siege_game.game_objects.states.state import State
from siege_game.game_objects.map.map import Map
from siege_game.game_objects.player import Player
import typing

class MapCommand(ABC):
    def __init__(self, sender:Player):
        self.__map:Map = Map.get_instance()
        self.__sender:Player = sender

    @abc.abstractmethod
    def execute(self, check:typing.Callable) -> bool:
        if check():
            return True
        
        else:
            return False
    
    @abc.abstractclassmethod
    def check(cls) -> bool: 
        raise NotImplementedError()
    

    

    
    
    