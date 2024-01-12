import random
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
        __special_actions (List[str]): The list of special actions the operator can perform.
        __weapon_holding (Weapons): The weapon the operator is holding.
        __mode_in (Modes): The mode the operator is in.

    Methods:
        get_hp() -> int: Returns the health points of the operator.
        get_inhand() -> List[str]: Returns the list of items in the operator's hand.
        get_sight_direction() -> int: Returns the direction the operator is facing.
        get_steps() -> int: Returns the number of steps the operator can take in a turn.
        get_weapon() -> Weapon: Returns the weapon the operator is holding.
        get_mode() -> Mode: Returns the mode the operator is in.
        set_hp(hp: int): Sets the health points of the operator.
        set_sight_direction(sight_direction: int): Sets the direction the operator is facing.
        set_steps(steps: int): Sets the number of steps the operator can take in a turn.
        set_inhand(inhand: List[str]): Sets the list of items in the operator's hand.
        set_weapon(weapon: Weapon): Sets the weapon the operator is holding.
        set_mode(mode: Mode): Sets the mode the operator is in.
    """
    def __init__(self, boss: Player) -> None:
        self.__boss = boss
        self.__name: str = None
        self.__hp: int = 100
        self.__inhand: str = None
        self.__weapon_holding: Weapons = None
        self.__mode_in: Modes = None
        self.__sight_direction: int = 0
        self.__steps: int = 5
        self.__special_actions: List[str] = ['switch_mode', 'throw_grenade', 'break', 'pre_aim']

    def get_hp(self) -> int:
        """
        Returns the health points of the operator.

        Returns:
            int: The health points of the operator.
        """
        return self.__hp

    def get_inhand(self) -> List[str]:
        """
        Returns the list of items in the operator's hand.

        Returns:
            List[str]: The list of items in the operator's hand.
        """
        return self.__inhand

    def get_sight_direction(self) -> int:
        """
        Returns the direction the operator is facing.

        Returns:
            int: The direction the operator is facing.
        """
        return self.__sight_direction

    def get_steps(self) -> int:
        """
        Returns the number of steps the operator can take in a turn.

        Returns:
            int: The number of steps the operator can take in a turn.
        """
        return self.__steps
    
    def get_weapon(self) -> Weapons:
        """
        Returns the weapon the operator is holding.

        Returns:
            Weapon: The weapon the operator is holding.
        """
        return self.__weapon_holding

    def get_mode(self) -> Modes:    
        """
        Returns the mode the operator is in.

        Returns:
            Mode: The mode the operator is in.
        """
        return self.__mode_in

    def set_hp(self, hp: int):
        """
        Sets the health points of the operator.

        Args:
            hp (int): The health points of the operator.
        """
        self.__hp = hp

    def set_sight_direction(self, sight_direction: int):
        """
        Sets the direction the operator is facing.

        Args:
            sight_direction (int): The direction the operator is facing.
        """
        self.__sight_direction = sight_direction
    
    def set_steps(self, steps: int):
        """
        Sets the number of steps the operator can take in a turn.

        Args:
            steps (int): The number of steps the operator can take in a turn.
        """
        self.__steps = steps

    def set_inhand(self, inhand: str):
        """
        Sets the list of items in the operator's hand.

        Args:
            inhand (List[str]): The list of items in the operator's hand.
        """
        self.__inhand = inhand

    def set_weapon(self, weapon: Weapons):
        """
        Sets the weapon the operator is holding.
        """
        self.__weapon_holding = weapon
    
    def set_mode(self, mode: Modes):
        """
        Sets the mode the operator is in.
        """
        self.__mode_in = mode