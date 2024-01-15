import uuid
from collections import deque
from siege_game.game_objects.commander import Commander

class Identity():
    ATTACK = uuid.uuid4()
    DEFEND = uuid.uuid4()

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

    Methods:
        
    """

    def __init__(self, name: str, identity:Identity, commander:Commander):
        """
        The constructor for the Player class.
        """

        self.__name = name
        self.__identity:Identity = identity
        self.__commander:Commander = commander

    def execute_command(self, command:str):
        self.__commander.execute_command(command)
          

