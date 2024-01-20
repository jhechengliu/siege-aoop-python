from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.battle import Battle
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity
from typing import List
from collections import deque
import json

class PlayerMovementBattleCommand(MapCommand):
    """
    command: move 3 3.5 4.5 => Try to move Attacker's 3rd operator to location x=3.5, y=4.5
    """
    logger = Logger("PlayerMovementBattleCommand")

    def execute(self) -> str:
        # calulate if movable, true/false
        if (self.get_send_player().get_identity() == Identity.ATTACK):
            # pointing same address, not copy
            operator:Operator = self.get_map().get_attacker(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
        elif (self.get_send_player().get_identity() == Identity.DEFEND):
            operator:Operator = self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])

        # check_sight(map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
        if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), [self.get_args[1](), self.get_args[2]()]) ):
            operator.set_location([self.get_args[1], self.get_args[2]])
            return "success"
        else:
            return "not-in-sight-error"
            # use dash (-) to connect heading
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), Battle):
            PlayerMovementBattleCommand.logger.error("move command can only be used in battle state")
            return "not-in-battle-state-error"
        elif (len(self.get_args()) != 3):
            PlayerMovementBattleCommand.logger.error(f"{type(self)}: Args len must be 3")
            return "args-len-error"

        return None
    
class PlayerCheckSightCommand(MapCommand):
    """
    will be call when: client selected an operator, use this operator as reference, and check if other operators are in sight
    command: check_sight 3 => check if Attacker's 3rd operator can see any defender's operator
    """
    logger = Logger("PlayerCheckSightCommand")

    def execute(self) -> None:
        """use json to send data"""
        if (self.get_send_player().get_identity() == Identity.ATTACK):
            # pointing same address, not copy
            operator:Operator = self.get_map().get_attacker(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            opponents:deque = self.get_map().get_defenders()
        elif (self.get_send_player().get_identity() == Identity.DEFEND):
            operator:Operator = self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            opponents:deque = self.get_map().get_attackers()

        # check_sight(map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
        msg = {}
        for i, opponent in enumerate(opponents):
            if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), opponent.get_location())):
                msg[str(i)] = 1
            else:
                msg[str(i)] = 0

        return json.dumps(msg) #convert from dictionary to json-formatted string


    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), Battle):
            PlayerCheckSightCommand.logger.error("check_sight command can only be used in battle state")
            return "not_in_battle_state_error"
        

        return None