from siege_game.game_objects.map.map_objects.map_object import MapObject

class Target(MapObject):
    """
    Defender have a target to protect and attackers' goal is to grab the target and escape.
    """
    def __init__(self, location:tuple) -> None:
        super().__init__(location)
        self.set_is_breakable(False)
        self.__is_taken = False

    def get_is_taken(self) -> bool:
        return self.__is_taken
    
    def set_is_taken(self, is_taken:bool) -> None:
        self.__is_taken = is_taken
    
    def __str__(self):
        return f"Entrance ({self.get_location()})"
    
    def __repr__(self):
        return f"Entrance ({self.get_location()})"