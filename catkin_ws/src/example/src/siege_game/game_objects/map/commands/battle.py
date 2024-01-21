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
        if (self.get_game().ready_count >= 1):
            # Start timer
            ReadyBattleCommand.logger.debug("PLAY")
            self.get_game().get_map().get_game_data_publisher().publish_client_A_server_actively("Play")
            self.get_game().get_map().get_game_data_publisher().publish_client_B_server_actively("Play")
            return "success"

        if (self.get_game().ready_count <= 0):
            self.get_game().ready_count += 1
            ReadyBattleCommand.logger.debug("GONNA TO PLAY")
            return "success"
        
        ReadyBattleCommand.logger.debug("BRUH")
        return "error"

    def check(self) -> bool:
        return None
    
class ClickBattleCommand(MapCommand):
    """
    command:click args:
    """
    logger = Logger("ClickBattleCommand")

    def execute(self) -> None:
        if (self.get_identity() == Identity.ATTACK):
            self.get_game().increment_attacker_click_count()
            return "success"
        elif (self.get_identity() == Identity.DEFEND):
            self.get_game().increment_defender_click_count()
            return "success"
        else:
            return "error"
        

    def check(self) -> bool:
        return None
    
class RequestBattleCommand(MapCommand):
    """
    command:click args:
    """
    logger = Logger("RequestBattleCommand")

    def execute(self) -> str:
        attacker_count = self.get_game().get_attacker_click_count()
        defender_count = self.get_game().get_defender_click_count()

        if (self.get_identity() == Identity.ATTACK):
            return f"success_{attacker_count}_{defender_count}"
        elif (self.get_identity() == Identity.DEFEND):
            return f"success_{defender_count}_{attacker_count}"
        else:
            return "error"
        

    def check(self) -> str:
        return None
    
