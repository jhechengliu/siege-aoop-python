from abc import ABC
import abc
from siege_game.game_objects.states.state import State
import typing

class MapCommand(ABC):
    @abc.abstractmethod
    def execute(self, check:typing.Callable) -> bool:
        raise NotImplementedError()
    
    @abc.abstractclassmethod
    def check(cls) -> bool: 
        raise NotImplementedError()
    

    

    
    
    