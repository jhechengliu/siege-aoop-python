import random
from enum import Enum
from typing import List, Tuple
from siege_game.game_objects.pawn.weapon import Weapon, Weapons
from siege_game.game_objects.pawn.mode import Mode, Modes
from siege_game.game_objects.constants.identity import Identity

class Operator():
    """
    """
    def __init__(self, location: List[float]) -> None:
        self.__hp: int = 100
        self.__weapon_holding:Weapons = Weapons.RIFLE
        self.__mode_in: Modes = Modes.NORMAL
        self.__sight_direction: int = 0
        self.__stemina: float = 5
        self.__location: List[float] = location
        self.__indentity: Identity = None
        self.__alive: bool = True

    def get_location(self) -> List[float]:
        return self.__location

    def set_location(self, location: List[float]) -> None:
        self.__location = location

    def get_hp(self) -> int:
        return self.__hp
    
    def set_hp(self, hp: int) -> None:
        self.__hp = hp
        if (self.__hp <= 0):
            self.__hp = 0
            self.__alive = False

    def get_identity(self) -> Identity:
        return self.__identity
    
    def is_alive(self) -> bool:
        return self.__alive

    def set_stemina(self, stemina: float) -> None:
        self.__stemina = stemina

    def get_stemina(self) -> float:
        return self.__stemina

    def get_weapon_holding(self) -> Weapons:
        return self.__weapon_holding
    

class SpecialActions(Enum):
    SWITCH_MODE: str = 'switch_mode'
    THROW_GRENADE: str = 'throw_grenade'
    BREAK: str = 'break'
    PRE_AIM: str = 'pre_aim'