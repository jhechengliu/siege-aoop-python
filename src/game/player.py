class Player():
    """
    This class represents a player in the game.
    It contains the name of the player and the identity of the player.
    The identity of the player is either Attacker or Defender.

    Attributes:
        name (str): The name of the player.
        identity (str): Whether Attacker or Defender.
        number_of_operators (int): The number of operators the player has.

    Methods:
        get_name() -> str: Returns the name of the player.
        set_name(name: str): Sets the name of the player.
        get_identity() -> str: Returns the identity of the player.
        set_identity(identity: str): Sets the identity of the player.
        get_number_of_operators() -> int: Returns the number of operators the player has.
        set_number_of_operators(number_of_operators: int): Sets the number of operators the player has.
    """

    def __init__(self, name: str):
        """
        The constructor for the Player class.

        Parameters:
            name (str): The name of the player.
            identity (str): Whether Attacker or Defender.
            number_of_operators (int): The number of operators the player has.
        """

        self.name = name
        self.identity = None
        self.number_of_operators = 0

    def get_name(self) -> str:
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """

        return self.name
    
    def set_name(self, name: str):
        """
        Sets the name of the player.

        Parameters:
            name (str): The name of the player.
        """

        self.name = name

    def get_identity(self) -> str:
        """
        Returns the identity of the player.

        Returns:
            str: The identity of the player.
        """

        return self.identity
    
    def set_identity(self, identity: str):
        """
        Sets the identity of the player.

        Parameters:
            identity (str): The identity of the player.
        """

        self.identity = identity

    def get_number_of_operators(self) -> int:
        """
        Returns the number of operators the player has.

        Returns:
            int: The number of operators the player has.
        """

        return self.number_of_operators
    
    def set_number_of_operators(self, number_of_operators: int):
        """
        Sets the number of operators the player has.

        Parameters:
            number_of_operators (int): The number of operators the player has.
        """

        self.number_of_operators = number_of_operators