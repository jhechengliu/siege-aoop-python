from typing import Callable, Tuple
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import StartState
from siege_game.game_objects.map.map import Map
import siege_game.game_objects.player as player
from siege_game.game_objects.constants.identity import Identity

import logging

class StartGameMapCommand(MapCommand):
    
    logger = logging.getLogger("StartGameMapCommand")

    def __init__(self, game, args:Tuple[str], identity:Identity):
        super().__init__(game, args, identity)
    
    def execute(self) -> None:
        self.get_map().get_game_flow_director().next_state()
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), StartState):
            StartGameMapCommand.logger.error(f"Game isn't in start state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return False
        elif (len(self.get_args()) != 0):
            StartGameMapCommand.logger.error("start command must have no args")
            return False
        else:
            return True