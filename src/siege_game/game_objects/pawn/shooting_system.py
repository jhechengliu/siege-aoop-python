from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.pawn.mode import Modes
import random

class ShootingSystem:
    """
    This class represents the shooting system in the game.
    It contains the shooting mechanics.

    Attributes:
        __accuracy (float): The accuracy of the operator. Depends on the mode of the operator.
        __damage_multiplier (float): The damage done to the target operator. Randomly generated (random.uniform(0, 1)).

    Methods:
        aim(operator: Operator) -> None: Base on the mode of the operator, the accuracy of the operator changes.
        shoot(operator: Operator) -> None: Calculates the damage done to the target operator, depending on the accuracy of the operator.
    """
    
    def __init__(self) -> None:
        """
        Initialize a new shooting system.
        """
        self.__accuracy: float = 1.0
        self.__damage_multiplier: float = 1.0

    def aim(self, operator: Operator) -> None:
        """
        Base on the mode of the operator, the accuracy of the operator changes.
        """
        if operator.get_mode() == Modes.NORMAL:
            self.__accuracy = random.uniform(0.3, 0.5)
        elif operator.get_mode() == Modes.AIMING:
            self.__accuracy = random.uniform(0.5, 0.8)
        elif operator.get_mode() == Modes.SHOOTING:
            self.__accuracy = random.uniform(0.8, 1.0)
        else:
            raise ValueError("Invalid mode")
        
    def shoot(self, operator: Operator) -> None:
        """
        Calculates the damage done to the target operator, depending on the accuracy of the operator.
        """
        self.__damage_multiplier = random.uniform(0, 1)
        operator.set_hp(operator.get_hp() - (self.__damage_multiplier * self.__accuracy * 10))
        if operator.get_hp() <= 0:
            operator.set_hp(0)
            operator.set_alive(False)