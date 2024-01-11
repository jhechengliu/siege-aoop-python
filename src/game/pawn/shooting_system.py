from operator import operator
import random

class ShootingSystem:
    """
    This class represents the shooting system in the game.
    It contains the shooting mechanics.

    Attributes:
        __accuracy (float): The accuracy of the operator. Depends on the mode of the operator.
        __damage_multiplier (float): The damage done to the target operator. Randomly generated.

    Methods:
        aim(operator: Operator) -> None: Base on the mode of the operator, the accuracy of the operator changes.
        shoot(operator: Operator) -> None: Calculates the damage done to the target operator, depending on the accuracy of the operator.
    """
    def __init__(self) -> None:
        """
        Initialize a new shooting system.
        """
        self.__accuracy: float = None
        self.__damage_multiplier: float = None