from siege_game.game_objects.pawn.operator import Operator

class Attacker(Operator):

    def __str__(self):
        return f"Attacker ({self.get_location()})"
    
    def __repr__(self):
        return f"Attacker ({self.get_location()})"