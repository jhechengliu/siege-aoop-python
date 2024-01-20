from abc import ABC
from typing import Tuple

class MapObject(ABC):

    def __init__(self, location:Tuple):
        self.__is_breakable = None
        self.__is_transparent = None
        self.__location = location

    def get_location(self) -> Tuple:
        return self.__location
    
    def set_is_breakable(self, is_breakable:bool) -> None:
        self.__is_breakable = is_breakable

    def get_is_breakable(self) -> Tuple:
        return self.__is_breakable
    
    def set_is_transparent(self, is_transparent:bool) -> None:
        self.__is_transparent = is_transparent

    def get_is_transparent(self) -> bool:
        return self.__is_transparent