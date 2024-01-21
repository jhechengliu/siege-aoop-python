from siege_game.game_objects.map.map_objects.map_object import MapObject

class Door(MapObject):
    def __init__(self, location:tuple):
        super().__init__(location)
        self.set_is_breakable(True)
        self.set_is_transparent(True)
        self.__is_broken = False
        self.__is_open = False

    def use(self) -> None:
        """
        open/close the door. If the door is open, close the door. If the door is closed, open the door.

        Returns: 
            (None)
        """
        if (self.__is_broken == False):
            self.__is_open = not self.__is_open
            self.__is_transparent = not self.__is_transparent
        else:
            self.__is_open = True

    def break_it(self) -> None:
        """
        break the door. If the door is still there, break the door. If door is dead, then do nothing.
        the door will turn into closed state and cannot be open because um...

        Returns: None
        """
        self.__is_broken = True
        self.__is_open = True
        self.__is_transparent = True
    
    def __str__(self):
        return f"Door ({self.get_location()})"
    
    def __repr__(self):
        return f"Door ({self.get_location()})"
    