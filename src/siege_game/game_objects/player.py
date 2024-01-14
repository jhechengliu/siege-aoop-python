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
        __identity (Identity): Whether Attacker or Defender.
        __total_number_of_operators (int): The number of operators the player has.
        __alive_operators (int): The number of operators the player has that are alive.

    Methods:
        
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
        self.__alive_operators = 0

class Identity():
    ATTACK = uuid.uuid4()
    DEFEND = uuid.uuid4()
