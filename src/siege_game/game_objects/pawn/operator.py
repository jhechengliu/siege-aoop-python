import random
from enum import Enum
from typing import List
from siege_game.game_objects.pawn.weapon import Weapon, Weapons
from siege_game.game_objects.pawn.mode import Mode, Modes
from siege_game.game_objects.pawn.shooting_system import ShootingSystem

class Operator():
    """
    """
    def __init__(self, location: List[float], shooting_system:ShootingSystem) -> None:
        self.__hp: int = 100
        self.__weapon_holding:Weapons = Weapons.Guns.RIFLE
        self.__mode_in: Modes = Modes.NORMAL
        self.__sight_direction: int = 0
        self.__steps: int = 5
        self.__location: List[float] = location
        self.__shooting_system = shooting_system

    def get_location(self) -> List[float]:
        return self.__location

    def shoot(self, target_opeartor) -> None:
        self.__shooting_system.aim(self)
        self.__shooting_system.shoot(target_opeartor)

class SpecialActions(Enum):
    SWITCH_MODE: str = 'switch_mode'
    THROW_GRENADE: str = 'throw_grenade'
    BREAK: str = 'break'
    PRE_AIM: str = 'pre_aim'