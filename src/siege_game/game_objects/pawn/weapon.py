from typing import List

class Weapon:
    """
    This class represents a weapon with a name and damage.

    Attributes:
        __name (str): The name of the weapon.
        __damage (int): The basic damage of the weapon.
        __required_hands (int): The number of hands required to hold the weapon.
        #也許之後加個遞減率之類的
    """
    def __init__(self, name: str, damage: int, __required_hands: int) -> None:
        """
        Initialize a new weapon.
        """
        self.__name = name
        self.__damage = damage
        self.__required_hands = __required_hands

class Weapons:
    """
    Enum for the different weapons.
    """
    RIFLE: Weapon = Weapon("rifle", 20, 2)
    PISTOL: Weapon = Weapon("pistol", 10, 1)
    KNIFE: Weapon = Weapon("knife", 100, 1)
    GRENADE: Weapon = Weapon("grenade", 70, 1)
    WEAPONS: List[Weapon] = [RIFLE, PISTOL, KNIFE, GRENADE]