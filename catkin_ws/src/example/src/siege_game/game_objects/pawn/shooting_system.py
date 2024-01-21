from siege_game.game_objects.pawn.mode import Modes
from siege_game.game_objects.pawn.operator import Operator
import random

class ShootingSystem:
    
    def __init__(self) -> None:
        self.__from_operator = None
        self.__to_operator = None

    def shoot(self, from_operator:Operator, to_operator:Operator) -> None:
        self.__from_operator = from_operator
        self.__to_operator = to_operator
        # cal damage
        pass