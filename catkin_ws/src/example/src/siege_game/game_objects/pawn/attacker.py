from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity

class Attacker(Operator):

    def __init__(self, location:list) -> None:
        super().__init__(location)
        self.__identity = Identity.ATTACK

    def get_identity(self) -> Identity:
        return self.__identity

    def __str__(self):
        return f"Attacker ({self.get_location()})"
    
    def __repr__(self):
        return f"Attacker ({self.get_location()})"