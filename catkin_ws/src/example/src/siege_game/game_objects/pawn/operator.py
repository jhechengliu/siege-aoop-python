import random
from enum import Enum
from typing import List, Tuple
from siege_game.game_objects.pawn.weapon import Weapon, Weapons
from siege_game.game_objects.pawn.mode import Mode, Modes

class Operator():
    """
    """
    def __init__(self, location: List[float]) -> None:
        self.__hp: int = 100
        self.__weapon_holding:Weapons = Weapons.Guns.RIFLE
        self.__mode_in: Modes = Modes.NORMAL
        self.__sight_direction: int = 0
        self.__steps: int = 5
        self.__location: List[float] = location

    def get_location(self) -> List[float]:
        return self.__location

    def set_location(self, location: List[float]) -> None:
        self.__location = location

    def get_hp(self) -> int:
        return self.__hp
    
    def set_hp(self, hp: int) -> None:
        self.__hp = hp

class SpecialActions(Enum):
    SWITCH_MODE: str = 'switch_mode'
    THROW_GRENADE: str = 'throw_grenade'
    BREAK: str = 'break'
    PRE_AIM: str = 'pre_aim'