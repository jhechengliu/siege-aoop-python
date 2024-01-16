from typing import List
from enum import Enum

class Weapon:
    """
    This class represents a weapon with a name and damage.

    Attributes:
        __name (str): The name of the weapon.
        __damage (int): The basic damage of the weapon.
        __required_hands (int): The number of hands required to hold the weapon.
    """
    def __init__(self, name: str, damage: int, __required_hands: int) -> None:
        """
        Initialize a new weapon.
        """
        self.__name = name
        self.__damage = damage
        self.__required_hands = __required_hands

class Weapons():
    """
    """
    class Guns(Enum):
        RIFLE: Weapon = Weapon("rifle", 20, 2)
        PISTOL: Weapon = Weapon("pistol", 10, 1)

    class Explosive(Enum):
        GRENADE: Weapon = Weapon("grenade", 70, 1)