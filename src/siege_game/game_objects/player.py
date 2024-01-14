import uuid

class Player():
    """
    This class represents a player in the game.
    A player is a real player playing the game
    A player can have multiple operators. In this game, we have 5 of them.
    It contains the name of the player and the identity of the player.
    The identity of the player is either Attacker or Defender.

    Attributes:
        __name (str): The name of the player.
        __identity (str): Whether Attacker or Defender.
        __total_number_of_operators (int): The number of operators the player has.
        __alive_operators (int): The number of operators the player has that are alive.

    Methods:
        get_name() -> str: Returns the name of the player.
        set_name(name: str): Sets the name of the player.
        get_identity() -> str: Returns the identity of the player.
        set_identity(identity: str): Sets the identity of the player.
        get_total_number_of_operators() -> int: Returns the number of operators the player has.
        set_total_number_of_operators(number_of_operators: int): Sets the number of operators the player has.
        get_alive_operators() -> int: Returns the number of operators the player has that are alive.
        set_alive_operators(alive_operators: int): Sets the number of operators the player has that are alive.
    """

    def __init__(self, name: str):
        """
        The constructor for the Player class.

        Parameters:
            name (str): The name of the player.
            identity (str): Whether Attacker or Defender.
            number_of_operators (int): The number of operators the player has.
        """

        self.__name = name
        self.__identity:Identity = None
        self.__number_of_operators = 0

    def get_name(self) -> str:
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """

        return self.__name
    
    def set_name(self, name: str):
        """
        Sets the name of the player.

        Parameters:
            name (str): The name of the player.
        """

        self.__name = name

    def get_identity(self) -> str:
        """
        Returns the identity of the player.

        Returns:
            str: The identity of the player.
        """

        return self.__identity
    
    def set_identity(self, identity: str):
        """
        Sets the identity of the player.

        Parameters:
            identity (str): The identity of the player.
        """

        self.__identity = identity

    def get_total_number_of_operators(self) -> int:
        """
        Returns the number of operators the player has.

        Returns:
            int: The number of operators the player has.
        """

        return self.__number_of_operators
    
    def set_total_number_of_operators(self, number_of_operators: int):
        """
        Sets the number of operators the player has.

        Parameters:
            number_of_operators (int): The number of operators the player has.
        """

        self.__number_of_operators = number_of_operators

    def get_alive_operators(self) -> int:
        """
        Returns the number of operators the player has that are alive.

        Returns:
            int: The number of operators the player has that are alive.
        """

        return self.__alive_operators
    
    def set_alive_operators(self, alive_operators: int):
        """
        Sets the number of operators the player has that are alive.

        Parameters:
            alive_operators (int): The number of operators the player has that are alive.
        """

        self.__alive_operators = alive_operators

class Identity():
    ATTACK = uuid.uuid4()
    DEFEND = uuid.uuid4()
