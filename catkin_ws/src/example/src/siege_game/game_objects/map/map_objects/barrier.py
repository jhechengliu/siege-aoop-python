from typing import Tuple
from siege_game.game_objects.map.map_objects.map_object import MapObject

class Barrier(MapObject):
    def __init__(self, location:Tuple):
        super().__init__(location)
        self.set_is_breakable(False)
    
    def __str__(self):
        return f"Barrier ({self.get_location()})"
    
    def __repr__(self):
        return f"Barrier ({self.get_location()})"