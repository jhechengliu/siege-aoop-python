from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.states.state import BattleState
from typing import List, Dict
from collections import deque
import json

        
class ReadyBattleCommand(MapCommand):
    """
    command: h:ready args:
    """
    logger = Logger("ReadyBattleSettingUpCommand")

    def execute(self) -> None:
        if (self.ready_count >= 1):
            # Start timer
            ReadyBattleCommand.logger.debug("PLAY")
            self.get_game().get_map().get_game_data_publisher().publish_client_A_server_actively("Play")
            self.get_game().get_map().get_game_data_publisher().publish_client_B_server_actively("Play")
            return "success"

        if (self.ready_count <= 0):
            ReadyBattleCommand.logger.debug("GONNA TO PLAY")
            self.ready_count = 1
            return "success"
        
        ReadyBattleCommand.logger.debug("BRUH")
        return "error"

    def check(self) -> bool:
        return None
    
