from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity

class Defender(Operator):

    def __init__(self, location:list) -> None:
        super().__init__(location)
        self.__identity = Identity.DEFEND

    def get_identity(self) -> Identity:
        return self.__identity
    
    def __str__(self):
        return f"Defender ({self.get_location()})"
    
    def __repr__(self):
        return f"Defender ({self.get_location()})"