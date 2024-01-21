from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.states.state import BattleState
from typing import List, Dict
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
            return "success" #success_stenima
        else:
            return "fail_block"
            # use dash (-) to connect heading
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerMovementBattleCommand.logger.error("move command can only be used in battle state")
            return "not-in-battle-state-error"
        elif (len(self.get_args()) != 2):
            PlayerMovementBattleCommand.logger.error(f"{type(self)}: Args len must be 2")
            return "args-len-error"

        return None
    
class PlayerCheckSightCommand(MapCommand):
    """
    will be call when: client selected an operator, use this operator as reference, and check if other operators are in sight
    command: checksight 3 => check if Attacker's 3rd operator can see any defender's operator
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
        in_sight_list:List = []
        for i, opponent in enumerate(opponents):
            if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), opponent.get_location())):
                in_sight_list.append(i)

        msg:dict = {"in_sight": in_sight_list}
        return "success_" + json.dumps(msg) #convert from dictionary to json-formatted string


    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerCheckSightCommand.logger.error("check_sight command can only be used in battle state")
            return "not_in_battle_state_error"
        
        return None
    
class PlayerShootCommand(MapCommand):
    """
    command: shoot 3 2 => X's 3rd operator shoot at Y's 2nd operator
    """ 
    logger = Logger("PlayerShootCommand")

    def execute(self) -> str:
        if (self.get_send_player().get_identity() == Identity.ATTACK):
            # pointing same address, not copy
            operator:Operator = self.get_map().get_attacker(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            opponent:Operator = self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])

        elif (self.get_send_player().get_identity() == Identity.DEFEND):
            operator:Operator = self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            opponent:Operator = self.get_map().get_attacker(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])

        # check_sight(map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
        if not (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), opponent.get_location())):
            return "not-in-sight-error"
        # else -> in sight, contunue. use shooting system to calculate damage
        # shoot sys, not implemented yet
        self.get_map().get_shooting_system().shoot(operator, opponent)
        return f"success_{str(opponent.get_hp())}"


    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerCheckSightCommand.logger.error(f"{type(self)}: check_sight command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 1):
            PlayerCheckSightCommand.logger.error(f"{type(self)}: Args len must be 1")
        
        return None
    
class MapUpdateCommand(MapCommand):
    """
    command: h:mapupdate, no args => send map data to client
    """
    logger = Logger("MapUpdateCommand")

    def execute(self) -> str:
        operators:List = []
        for i, operator in self.get_map().get_attackers():
            map_operator_data:dict = {}
            map_operator_data["index"] = i
            map_operator_data["location"] = operator.get_location()
            map_operator_data["identity"] = "attacker"
            operators.append(map_operator_data)
        for i, operator in self.get_map().get_defenders():
            map_operator_data:dict = {}
            map_operator_data["index"] = i
            map_operator_data["location"] = operator.get_location()
            map_operator_data["identity"] = "defender"
            operators.append(map_operator_data)
        
        msg:dict = {"player": operators}
        msg = json.dumps(msg)
        
        return f"success_" + msg

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            MapUpdateCommand.logger.error("mapupdate command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 0):
            MapUpdateCommand.logger.error("mapupdate command must have no args")
            return "args_len_error"
        
        return None