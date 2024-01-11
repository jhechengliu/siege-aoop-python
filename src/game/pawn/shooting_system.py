from operator import operator
import random

class ShootingSystem:
    """
    This class represents the shooting system in the game.
    It contains the shooting mechanics.

    Attributes:
        accuracy (int): The accuracy of the operator. Depends on the mode of the operator.
        damage (int): The damage done to the target operator. Randomly generated.

    Methods:
        aim(operator: Operator) -> None: Base on the mode of the operator, the accuracy of the operator changes.
        shoot(operator: Operator) -> None: Calculates the damage done to the target operator, depending on the accuracy of the operator.
    """