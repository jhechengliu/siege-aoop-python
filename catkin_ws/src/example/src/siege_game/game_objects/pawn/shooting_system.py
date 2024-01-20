from siege_game.game_objects.pawn.mode import Modes
import random

class ShootingSystem:
    
    def __init__(self) -> None:
        pass

    # to call, pass: 1.current map, 2.ATT-operator 3.DEF-operator
    # use sight_checker to check if the target is in sight
    # then calulate damage