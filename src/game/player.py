class Player():
    """
    This class represents a player in the game.
    It contains the name of the player and the identity of the player.
    The identity of the player is either Attacker or Defender.

    Attributes:
        name (str): The name of the player.
        identity (str): Whether Attacker or Defender.

    Methods:
        get_name() -> str: Returns the name of the player.
        get_identity() -> str: Returns the identity of the player.
        set_name(name: str): Sets the name of the player.
    """
    def __init__(self, name: str):
        """
        The constructor for the Player class.

        Parameters:
            name (str): The name of the player.
        """

        self.name = name
        self.identity = None

    def get_name(self) -> str:
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """

        return self.name
    
    def get_identity(self) -> str:
        """
        Returns the identity of the player.

        Returns:
            str: The identity of the player.
        """

        return self.identity
    
    def set_name(self, name: str):
        """
        Sets the name of the player.

        Parameters:
            name (str): The name of the player.
        """

        self.name = name
    
    def __str__(self) -> str:
        """
        Returns a string representation of the player.

        Returns:
            str: The string representation of the player.
        """

        return self.name + ": " + str(self.score) + " points"

    def __init__(self, name: str):
        """
        The constructor for the Player class.

        Parameters:
            name (str): The name of the player.
        """

        self.name = name
        self.get_identity = None
    
    def get_name(self) -> str:
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """

        return self.name
    
    def get_identity(self) -> str:
        """
        Returns the identity of the player.

        Returns:
            str: The identity of the player.
        """

        return self.identity
    
    def __str__(self):
        """
        Returns a string representation of the player.

        Returns:
            str: The string representation of the player.
        """

        return self.name + ": " + str(self.score) + " points"