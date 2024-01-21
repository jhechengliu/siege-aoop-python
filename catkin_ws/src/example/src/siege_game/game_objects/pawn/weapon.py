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

    def get_name(self) -> str:
        return self.__name
    
    def get_damage(self) -> int:
        return self.__damage
    
    def get_required_hands(self) -> int:
        return self.__required_hands

class Weapons(Enum):
    RIFLE: Weapon = Weapon("rifle", 20, 2)
    PISTOL: Weapon = Weapon("pistol", 10, 1)
    GRENADE: Weapon = Weapon("grenade", 70, 1)

    def get_name(self) -> str:
        return self.value.get_name()
    
    def get_damage(self) -> int:
        return self.value.get_damage()
    
    def get_required_hands(self) -> int:
        return self.value.get_required_hands()
      