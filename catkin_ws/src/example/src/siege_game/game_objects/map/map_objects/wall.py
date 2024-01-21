from typing import Tuple
from siege_game.game_objects.map.map_objects.map_object import MapObject

class Wall(MapObject):
    def __init__(self, location: Tuple) -> None:
        super().__init__(location)
        self.set_is_breakable(False)
        self.set_is_transparent(False)
    
    def __str__(self):
        return f"Wall ({self.get_location()})"
    
    def __repr__(self):
        return f"Wall ({self.get_location()})"