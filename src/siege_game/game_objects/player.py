from siege_game.game_objects.constants.identity import Identity
import siege_game.game_objects.commander as commander_file

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

    def __init__(self, name: str, identity:Identity, commander):
        """
        The constructor for the Player class.
        """

        self.__name = name
        self.__identity:Identity = identity
        self.__commander:commander_file.Commander = commander

    def execute_command(self, command:str):
        self.__commander.execute_command(command)
          

