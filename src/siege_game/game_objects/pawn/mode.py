from typing import List
from enum import Enum

class Mode:
    """
    Mode of a operator.

    Variables:
        __sight_distance (int): The maximum distance the operator can see.
        __sight_angle (int): The angle of vision for the operator.
    """

    def __init__(self, sight_distance: int, sight_angle: int):
        """
        Constructor for the Mode class.
        """
        self.__sight_distance: int = sight_distance
        self.__sight_angle: int = sight_angle

class Modes(Enum):
    """
    Enum for the different modes of the operator.
    """    
    NORMAL: Mode = Mode(5, 360)
    AIM_DOWN_SIGHT: Mode = Mode(10, 45)