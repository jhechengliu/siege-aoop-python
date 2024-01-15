from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.pawn.mode import Modes
import random

class ShootingSystem:
    """
    
    """
    
    def __init__(self) -> None:
        """
        Initialize a new shooting system.
        """
        self.__accuracy: float = 1.0
        self.__damage: float = 1.0

    def aim(self, operator: Operator) -> None:
        """
        Base on the mode of the operator, the accuracy of the operator changes.
        """
        if operator.get_mode() == Modes.NORMAL:
            self.__accuracy = random.uniform(0.5, 0.8)
        elif operator.get_mode() == Modes.AIM_DOWN_SIGHT:
            self.__accuracy = random.uniform(0.8, 1.0)
        else:
            raise ValueError("Invalid mode")
        
    def shoot(self, operator: Operator) -> None:
        """
        Calculates the damage done to the target operator, depending on the accuracy of the operator.
        """
        self.__damage = operator.get_weapon().get_damage()
        operator.set_hp(operator.get_hp() - (self.__damage * self.__accuracy))
        if operator.get_hp() <= 0:
            operator.set_hp(0)
            operator.set_alive(False)