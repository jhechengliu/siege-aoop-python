class Floor():
    """
    Its just a floor

    Attributes:
        __location (tuple): the location of the floor

    Methods:
        get_location: the getter of the __location attribute
    """
    def __init__(self, location:tuple):
        self.__location = location

    def get_location(self) -> tuple:
        """
        Returns: 
            (tuple) location of the floor 
        """
        return self.__location