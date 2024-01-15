from siege_game.game_objects.pawn.operator import Operator

class Defender(Operator):
    def __str__(self):
        return f"Defender ({self.get_location()})"
    
    def __repr__(self):
        return f"Defender ({self.get_location()})"