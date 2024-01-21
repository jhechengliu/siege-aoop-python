from siege_game.game_objects.constants.identity import Identity
import siege_game.game_objects.commander as commander_file

class Player():

    def __init__(self, name: str, identity:Identity, commander):
        """
        The constructor for the Player class.
        """

        self.__name = name
        self.__identity:Identity = identity
        self.__commander:commander_file.Commander = commander
        self.__has_finish_setting_up = False # redundant but i already wrote some func else so i'll just leave it here

    def execute_command(self, command:str) -> str:
        return self.__commander.execute_command(command, self)

    def get_identity(self):
        return self.__identity
    
    def set_has_finish_setting_up(self, has_finish_setting_up:bool):
        self.__has_finish_setting_up = has_finish_setting_up

    def get_has_finish_setting_up(self):
        return self.__has_finish_setting_up
    
    def get_name(self):
        return self.__name

    def __str__(self):
        identity_str = ""
        if (self.__identity == Identity.ATTACK):
            identity_str = "Attack"
        elif (self.__identity == Identity.DEFEND):
            identity_str = "Defend"

        return f"Player {self.__name}, Identity: {identity_str}"
    
    def __repr__(self):
        identity_str = ""
        if (self.__identity == Identity.ATTACK):
            identity_str = "Attack"
        elif (self.__identity == Identity.DEFEND):
            identity_str = "Defend"

        return f"Player {self.__name}, Identity: {identity_str}"