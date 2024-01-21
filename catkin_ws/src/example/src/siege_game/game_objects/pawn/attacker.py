from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity

class Attacker(Operator):

    def __init__(self, name:str, location:tuple, health:int, attack:int, defense:int, speed:int, mode:str) -> None:
        super().__init__(name, location, health, attack, defense, speed, mode)
        self.__identity = Identity.ATTACK

    def __str__(self):
        return f"Attacker ({self.get_location()})"
    
    def __repr__(self):
        return f"Attacker ({self.get_location()})"