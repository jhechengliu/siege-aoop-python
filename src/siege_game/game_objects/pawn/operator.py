import random
from enum import Enum
from typing import List
from siege_game.game_objects.player import Player
from siege_game.game_objects.pawn.weapon import Weapon, Weapons
from siege_game.game_objects.pawn.mode import Mode, Modes

class Operator():
    """
    This class represents an operator in the game.
    An operator is a pawn in the game.
    #It contains the name of the operator and the type of the operator.
    #The type of the operator is either Attacker or Defender.
    #A player has five operators.

    Attributes:
        __boss (Player): The player that owns the operator.
        __name (str): The name of the operator. #In R6 has many operators.
        __hp (int): The health points of the operator.
        __inhand (List[str]): The list of items in the operator's hand. Weapons, gadgets, etc.
        __sight_direction (int): The direction the operator is facing.
        __steps (int): The number of steps the operator can take in a turn.
        __weapon_holding (Weapons): The weapon the operator is holding.
        __mode_in (Modes): The mode the operator is in.
        __location (List[float]): The location of the operator.

    Methods:
    """
    def __init__(self, boss: Player, location: list[int]) -> None:
        self.__boss = boss
        self.__name: str = boss.get_identity()
        self.__hp: int = 100
        self.__inhand: str = None
        self.__weapon_holding: Weapons = None
        self.__mode_in: Modes = Modes.NORMAL
        self.__sight_direction: int = 0
        self.__steps: int = 5
        self.__location: list[float] = location

class SpecialActions(Enum):
    SWITCH_MODE: str = 'switch_mode'
    THROW_GRENADE: str = 'throw_grenade'
    BREAK: str = 'break'
    PRE_AIM: str = 'pre_aim'