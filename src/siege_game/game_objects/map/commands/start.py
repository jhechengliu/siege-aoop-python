from typing import Callable
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import StartState
from siege_game.game_objects.map.map import Map
import logging

class StartGameMapCommand(MapCommand):
    
    logger = logging.getLogger("StartGameMapCommand")

    def __init__(self, map:Map, args:tuple[str]):
        self.__init__(self, map, args)

    
    def execute(self) -> bool:
        self.get_map().get_game_flow_director().next_state()
        
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), StartState):
            StartGameMapCommand.logger.error("Game isn't in start state. Hence, you cant use start command")
            return False
        elif (len(self.get_args()) != 0):
            StartGameMapCommand.logger.error("start command must have no args")
            return False
        else:
            return True