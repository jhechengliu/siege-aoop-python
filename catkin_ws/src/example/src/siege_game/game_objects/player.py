from siege_game.game_objects.constants.identity import Identity
import siege_game.game_objects.commander as commander_file

class Player():
    """
    The Player class represents a player in the Siege game. It handles player identity and commands.

    Attributes:
        __name: The name of the player.
        __identity: The identity of the player (attacker or defender).
        __commander: The Commander object that handles game commands for this player.
        __has_finish_setting_up: A boolean flag indicating whether the player has finished setting up.

    Methods:
        execute_command(command:str): Executes a game command.
        get_identity(): Returns the identity of the player.
        set_has_finish_setting_up(has_finish_setting_up:bool): Sets the setup status of the player.
        get_has_finish_setting_up(): Returns the setup status of the player.
        get_name(): Returns the name of the player.
        __str__(): Returns a string representation of the player.
        __repr__(): Returns a string representation of the player.
    """
    def __init__(self, name: str, identity:Identity, commander):
        """
        Initializes a new instance of the Player class.

        Args:
            name (str): The name of the player.
            identity (Identity): The identity of the player (attacker or defender).
            commander: The Commander object that handles game commands for this player.
        """
        self.__name = name
        self.__identity:Identity = identity
        self.__commander:commander_file.Commander = commander
        self.__has_finish_setting_up = False # redundant but i already wrote some func else so i'll just leave it here

    def execute_command(self, command:str) -> str:
        """
        Executes a game command.

        Args:
            command (str): The game command to execute.

        Returns:
            str: The result of the command execution.
        """
        return self.__commander.execute_command(command, self)

    def get_identity(self):
        """
        Returns the identity of the player.

        Returns:
            Identity: The identity of the player (attacker or defender).
        """
        return self.__identity
    
    def set_has_finish_setting_up(self, has_finish_setting_up:bool):
        """
        Sets the setup status of the player.

        Args:
            has_finish_setting_up (bool): The setup status of the player.
        """
        self.__has_finish_setting_up = has_finish_setting_up

    def get_has_finish_setting_up(self):
        """
        Returns the setup status of the player.

        Returns:
            bool: The setup status of the player.
        """
        return self.__has_finish_setting_up
    
    def get_name(self):
        """
        Returns the name of the player.

        Returns:
            str: The name of the player.
        """
        return self.__name

    def __str__(self):
        """
        Returns a string representation of the player.

        Returns:
            str: A string representation of the player.
        """
        identity_str = ""
        if (self.__identity == Identity.ATTACK):
            identity_str = "Attack"
        elif (self.__identity == Identity.DEFEND):
            identity_str = "Defend"

        return f"Player {self.__name}, Identity: {identity_str}"
    
    def __repr__(self):
        """
        Returns a string representation of the player.

        Returns:
            str: A string representation of the player.
        """
        identity_str = ""
        if (self.__identity == Identity.ATTACK):
            identity_str = "Attack"
        elif (self.__identity == Identity.DEFEND):
            identity_str = "Defend"

        return f"Player {self.__name}, Identity: {identity_str}"