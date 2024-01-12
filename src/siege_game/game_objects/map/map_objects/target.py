class Target:
    """
    Defender have a target to protect and attackers' goal is to grab the target and escape.
    """
    def __init__(self, location:tuple) -> None:
        self.__location: tuple = location

    def get_location(self) -> tuple:
        return self.__location
    
    def __str__(self):
        return f"Entrance ({self.__location})"
    
    def __repr__(self):
        return f"Entrance ({self.__location})"