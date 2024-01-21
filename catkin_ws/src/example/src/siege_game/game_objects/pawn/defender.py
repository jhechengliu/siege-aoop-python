from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity

class Defender(Operator):

    def __init__(self, name:str, location:tuple, health:int, attack:int, defense:int, speed:int, mode:str) -> None:
        super().__init__(name, location, health, attack, defense, speed, mode)
        self.__identity = Identity.DEFEND
    
    def __str__(self):
        return f"Defender ({self.get_location()})"
    
    def __repr__(self):
        return f"Defender ({self.get_location()})"