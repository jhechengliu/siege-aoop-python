from typing import Callable, Tuple
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import StartState
from siege_game.game_objects.map.map import Map
import siege_game.game_objects.player as player
from siege_game.game_objects.constants.identity import Identity

from siege_game.game_objects.logger import Logger

class StartGameMapCommand(MapCommand):
    
    logger = Logger("StartGameMapCommand")

    def __init__(self, game, args:Tuple[str], identity:Identity):
        super().__init__(game, args, identity)
    
    def execute(self) -> None:
        self.get_map().get_game_flow_director().next_state()
        return "success"
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), StartState):
            StartGameMapCommand.logger.error(f"Game isn't in start state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return "not_in_start_state_error"
        elif (len(self.get_args()) != 0):
            StartGameMapCommand.logger.error("start command must have no args")
            return "args_len_error"
        else:
            return None