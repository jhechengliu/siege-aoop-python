from siege_game.game_objects.map.map import Map
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.start import StartGameMapCommand
from siege_game.game_objects.map.commands.setting_up import SetOperatorSettingUpCommand, FinishSettingUpCommand
from siege_game.game_objects.map.commands.battle import ReadyBattleCommand, ClickBattleCommand, RequestBattleCommand
from siege_game.game_objects.player import Player
from siege_game.game_objects.logger import Logger

class Commander():
    """
    Commander breaks down the command and execute the command.
    """
    
    logger = Logger("Commander")

    def __init__(self, game):
        from siege_game.game import Game
        self.__game:Game = game
        self.__command_headings = {
            "start": StartGameMapCommand,
            "setoperator": SetOperatorSettingUpCommand,
            "finishsettingup": FinishSettingUpCommand,
            "ready": ReadyBattleCommand,
            "click": ClickBattleCommand,
            "request": RequestBattleCommand
        }
    def execute_command(self, command:str, player:Player) -> str:
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
