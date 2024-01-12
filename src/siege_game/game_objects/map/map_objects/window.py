class Window():
    def __init__(self, location:tuple):
        """
            Initialize the object with a location attribute.

            Args:
                location (tuple): A pair of coordinates (x, y) representing the location of the object.

            Attributes:
                __isbroken (bool): A private attribute to indicate if the object is broken. Default to False.
                __location (tuple): A private attribute to store the location of the object.
        """
        self.__isbroken = False
        self.__location = location
    
    def get_location(self) -> tuple:
        """
        Returns: 
            (tuple) location of the window
        """
        return self.__location
    
    def __str__(self):
        return f"Window ({self.__location})"
    
    def __repr__(self):
        return f"Window ({self.__location})"