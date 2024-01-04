class Player():
    """
    This class represents a player in the game.

    Attributes:
        name (str): The name of the player.
        identity (str): Whether Attacker or Defender.

    Methods:
        get_name(): Returns the name of the player.
        get_identity(): Returns the identity of the player.
    """

    def __init__(self, name):
        """
        The constructor for the Player class.

        Parameters:
            name (str): The name of the player.
        """

        self.name = name
        self.get_identity = []
    
    def get_name(self):
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """

        return self.name
    
    def get_identity(self):
        """
        Returns the identity of the player.

        Returns:
            list: The identity of the player.
        """

        return self.identity
    
    def __str__(self):
        """
        Returns a string representation of the player.

        Returns:
            str: The string representation of the player.
        """

        return self.name + ": " + str(self.score) + " points"