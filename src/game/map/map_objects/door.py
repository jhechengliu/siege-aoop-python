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
        self.__is_open = not self.__is_open

    def break_it(self):
        self.__is_broken = True