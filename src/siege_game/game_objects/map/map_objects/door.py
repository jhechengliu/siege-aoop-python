class Door():
    """
    Well its a door

    Attributes:
        __is_open (boolean): true when door is in open state. False when door is closed.
        __is_broken (boolean): true when door flew away, false when door is still there.
        __location (boolean): the location of the door in the map, represented using tuple (y, x)

    Methods:
        use: open/close the door
        break_it: break the door
        get_location: the getter of the door's location
    """
    def __init__(self, location:tuple):
        """
        Attributes:
            location: the location of the newly made door
        """
        self.__is_open = False
        self.__is_broken = False
        self.__location = location

    def use(self) -> None:
        """
        open/close the door. If the door is open, close the door. If the door is closed, open the door.

        Returns: 
            (None)
        """
        if (self.__is_broken == False):
            self.__is_open = not self.__is_open
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

    def get_location(self) -> tuple:
        """
        Returns: 
            (tuple) the location of the door object
        """
        return self.__location
    
    def __str__(self):
        return f"Door ({self.__location})"
    
    def __repr__(self):
        return f"Door ({self.__location})"
    