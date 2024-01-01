class Door():
    """
    Well its a door

    Attributes:
        __is_open: true when door is in open state. False when door is closed.
        __is_broken: true when door flew away, false when door is still there.

    Methods:
        use: open/close the door
        break_it: break the door
    """
    def __init__(self):
        self.__is_open = False
        self.__is_broken = False

    def use(self):
        """
        open/close the door. If the door is open, close the door. If the door is closed, open the door.
        """
        if (self.__is_broken == False):
            self.__is_open = not self.__is_open
        else:
            self.__is_open = True

    def break_it(self):
        """
        break the door. If the door is still there, break the door. If door is dead, then do nothing.
        the door will turn into closed state and cannot be open because um...
        """
        self.__is_broken = True
        self.__is_open = True