from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.start import StartGameMapCommand
from siege_game.game_objects.map.commands.battle import ReadyBattleCommand, ClickBattleCommand, RequestBattleCommand, WhoWinBattleCommand
from siege_game.game_objects.logger import Logger

class Commander():
    """
    The Commander class is responsible for breaking down and executing commands in the game.

    Attributes:
        logger: A Logger object to log information about the game.
        __game: A Game object representing the current game.
        __command_headings: A dictionary mapping command headings to their corresponding command classes.

    Methods:
        execute_command(command:str, player:Player) -> str:
            Executes a command for a player and returns the result.
    """
    
    logger = Logger("Commander")

    def __init__(self, game):
        from siege_game.game import Game
        self.__game:Game = game
        self.__command_headings = {
            "start": StartGameMapCommand,
            "ready": ReadyBattleCommand,
            "click": ClickBattleCommand,
            "request": RequestBattleCommand,
            "whowin": WhoWinBattleCommand
        }
    def execute_command(self, command:str, player) -> str:
        """
        Executes a command for a player and returns the result.

        The command is split into a heading and arguments. The heading is used to look up the corresponding command class in __command_headings.
        A new instance of the command class is created and its check() method is called. If check() returns None, the command is executed and its result is returned.
        If check() returns anything else, the command is denied and the denial reason is returned.

        Args:
            command (str): The command to execute. Should be in the format "<heading> <args>", where <heading> is the command heading and <args> are the command arguments.
            player (Player): The player for whom to execute the command.

        Returns:
            str: The result of the command, or a denial reason if the command was denied.

        Raises:
            Warning: If the command heading is not found in __command_headings.
        """
        Commander.logger.info(f"{type(self)}: Executing Command")
        command_list = command.split()
        command_heading = command_list[0]
        command_args = None
        if (len(command_list) > 1):
            command_args = tuple(command_list[1:])
        else:
            command_args = tuple()

        Commander.logger.info(f"Command heading:{command_heading} ({type(command_heading)}), Command args:{command_args} ({type(command_args)})")

        if (command_heading in self.__command_headings.keys()):
            command:MapCommand = self.__command_headings[command_heading](self.__game, command_args, player)
            
            commend_check_result = command.check()
            if (commend_check_result == None):
                Commander.logger.debug(f"{type(self)}: Command Successfully Executed")
                return command.execute()
            else:
                Commander.logger.error(f"{type(self)}: Command Execute denied: {commend_check_result}")
                return commend_check_result
            
        else:
            Commander.logger.warning("Command Heading Not Found. Do you forgot to add headings into the __command_headings?")
            return "command_not_found_error"
