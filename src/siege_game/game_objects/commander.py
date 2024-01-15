from siege_game.game_objects.map.map import Map
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.start import StartGameMapCommand
from siege_game.game_objects.map.commands.setting_up import InitPlayerSettingUpCommand
from siege_game.game_objects.constants.identity import Identity
import logging

class Commander():
    logger = logging.getLogger("Commander")

    def __init__(self, map:Map):
        self.__map = map
        self.__command_headings = {
            "start": StartGameMapCommand
        }
    def execute_command(self, command:str, identity:Identity):
        Commander.logger.info("Executing Command")
        command_list = command.split()
        command_heading = command_list[0]
        command_args = None
        if (len(command_list) > 1):
            command_args = tuple(command_list[1:])
        else:
            command_args = tuple()

        Commander.logger.info(f"Command heading:{command_heading} ({type(command_heading)}), Command args:{command_args} ({type(command_args)})")

        if (command_heading in self.__command_headings.keys()):
            command:MapCommand = self.__command_headings[command_heading](self.__map, command_args, identity)
            if (command.check()):
                command.execute()
            
        else:
            Commander.logger.warn("Command Heading Not Found. Do you forgot to add headings into the __command_headings?")
