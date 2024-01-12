class Wall():
    
    def __init__(self, location: tuple) -> None:
        """
            Initialize the object with a location attribute.

            Args:
                location (tuple): A pair of coordinates (x, y) representing the location of the object.

            Attributes:
                __location (tuple): A private attribute to store the location of the object.
        """
        self.__location = location

    def get_location(self) -> tuple:
        """
        Returns: 
            (tuple) location of the wall
        """
        return self.__location
    
    def __str__(self):
        return f"Wall ({self.__location})"
    
    def __repr__(self):
        return f"Wall ({self.__location})"