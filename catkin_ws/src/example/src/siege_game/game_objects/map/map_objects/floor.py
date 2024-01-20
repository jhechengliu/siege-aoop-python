from siege_game.game_objects.map.map_objects.map_object import MapObject

class Floor(MapObject):
    def __init__(self, location:tuple):
        super().__init__(location)
        self.set_is_breakable(False)
        self.set_is_transparent(True)
    
    def __str__(self):
        return f"Floor ({self.get_location()})"
    
    def __repr__(self):
        return f"Floor ({self.get_location()})"