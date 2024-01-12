class Entrance():
    def __init__(self, location:tuple):
        self.__location = location

    def get_location(self) -> tuple:
        return self.__location
    
    def __str__(self):
        return f"Entrance ({self.__location})"
    
    def __repr__(self):
        return f"Entrance ({self.__location})"