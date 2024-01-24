from typing import Callable, Tuple
from siege_game.game_objects.map.commands.map_command import MapCommand
import siege_game.game_objects.player as player
from siege_game.game_objects.constants.identity import Identity

from siege_game.game_objects.logger import Logger

class StartGameMapCommand(MapCommand):
    
    logger = Logger("StartGameMapCommand")

    def __init__(self, game, args:Tuple[str], identity:Identity):
        super().__init__(game, args, identity)
    
    def execute(self) -> None:
        return "success"
    
    def check(self) -> bool:
        if (len(self.get_args()) != 0):
            StartGameMapCommand.logger.error("start command must have no args")
            return "args_len_error"
        else:
            return None