class Barrier():
    def __init__(self, location:tuple):
        self.__location = location

    def get_location(self) -> tuple:
        return self.__location
    
    def __str__(self):
        return f"Barrier ({self.__location})"
    
    def __repr__(self):
        return f"Barrier ({self.__location})"