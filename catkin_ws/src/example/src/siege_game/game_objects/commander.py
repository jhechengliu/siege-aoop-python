from siege_game.game_objects.map.map import Map
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.start import StartGameMapCommand
from siege_game.game_objects.map.commands.setting_up import SetOperatorSettingUpCommand, FinishSettingUpCommand, StartBattleSettingUpCommand
from siege_game.game_objects.player import Player
from siege_game.game_objects.logger import Logger

class Commander():
    logger = Logger("Commander")

    def __init__(self, game):
        from siege_game.game import Game
        self.__game:Game = game
        self.__command_headings = {
            "start": StartGameMapCommand,
            "setoperator": SetOperatorSettingUpCommand,
            "finishsettingup": FinishSettingUpCommand,
            "startbattle": StartBattleSettingUpCommand
        }
    def execute_command(self, command:str, player:Player):
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
            command:MapCommand = self.__command_headings[command_heading](self.__game, command_args, player)
            if (command.check()):
                command.execute()
            
        else:
            Commander.logger.warn("Command Heading Not Found. Do you forgot to add headings into the __command_headings?")
