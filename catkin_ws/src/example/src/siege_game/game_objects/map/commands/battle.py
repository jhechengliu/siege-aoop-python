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
            operator:Operator = self.get_map().get_attacker(self.get_args[0])
        elif (self.get_send_player().get_identity() == Identity.DEFEND):
            operator:Operator = self.get_map().get_defender(self.get_args[0])

        if not (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), [self.get_args[1](), self.get_args[2]()]) ):
            return "fail_block"
            
        # check if go through window, if so, minus 1 more stemina. if stemina < 0, return fail_stemina (early return)
        distance:float = self.get_map().get_sight_checker().calculate_distance(operator.get_location(), [self.get_args[1], self.get_args[2]])
        stemina_cost:float = distance
        if (self.get_map().get_sight_checker().check_if_go_through_window(self.get_map().get_map_data(), operator.get_location(), [self.get_args[1], self.get_args[2]])):
            stemina_cost += 1
        if (operator.get_stemina() - stemina_cost < 0):
            return "fail_stemina"
        
        # check_sight(map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
        
        operator.set_location([self.get_args[1], self.get_args[2]])
        operator.set_stemina(operator.get_stemina() - stemina_cost)
        PlayerCheckSightBattleCommand.logger.info(f"{type(self)}: {operator.get_identity()}'s {operator.get_index()}th operator moved to {operator.get_location()}, stemina left: {operator.get_stemina()}")
        return "success" #success_stenima
    
    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerMovementBattleCommand.logger.error("move command can only be used in battle state")
            return "not-in-battle-state-error"
        elif (len(self.get_args()) != 2):
            PlayerMovementBattleCommand.logger.error(f"{type(self)}: Args len must be 2")
            return "args-len-error"

        return None
    
class PlayerCheckSightBattleCommand(MapCommand):
    """
    will be call when: client selected an operator, use this operator as reference, and check if other operators are in sight
    command: h:checksight arg:3 => check if Attacker's 3rd operator can see any defender's operator
    """
    logger = Logger("PlayerCheckSightCommand")

    def execute(self) -> str:
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
        for i in range (len(opponents)):
            opponent = opponents[i]
            if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), opponent.get_location())):
                in_sight_list.append(i)

        msg:dict = {"in_sight": in_sight_list}
        return "success_" + json.dumps(msg).strip() #convert from dictionary to json-formatted string


    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerCheckSightBattleCommand.logger.error("check_sight command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 1):
            PlayerCheckSightBattleCommand.logger.error(f"{type(self)}: Args len must be 1")
            return "args_len_error"
        
        return None
    
class PlayerShootBattleCommand(MapCommand):
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
            PlayerCheckSightBattleCommand.logger.error(f"{type(self)}: check_sight command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 1):
            PlayerCheckSightBattleCommand.logger.error(f"{type(self)}: Args len must be 1")
        
        return None
    
class MapUpdateBattleCommand(MapCommand):
    """
    command: h:mapupdate, no args => send map data to client
    """
    logger = Logger("MapUpdateCommand")

    def execute(self) -> str:
        operators:List = []
        for i in range (0, len(self.get_map().get_attackers())):
            operator = self.get_map().get_attacker(i)
            map_operator_data:dict = {}
            map_operator_data["index"] = i
            map_operator_data["location"] = operator.get_location()
            map_operator_data["identity"] = "attacker"
            operators.append(map_operator_data)
        for i in range (0, len(self.get_map().get_defenders())):
            operator = self.get_map().get_defender(i)
            map_operator_data["index"] = i
            map_operator_data["location"] = operator.get_location()
            map_operator_data["identity"] = "defender"
            operators.append(map_operator_data)
        
        msg:dict = {"player": operators}
        msg = json.dumps(msg).strip()
        MapUpdateBattleCommand.logger.info(f"{type(self)}: JSON formmated string to be sent: {msg}")
        return f"success_" + msg

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            MapUpdateBattleCommand.logger.error("mapupdate command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 0):
            MapUpdateBattleCommand.logger.error("mapupdate command must have no args")
            return "args_len_error"
        
        return None
    
class BattleFlowBattleCommander(MapCommand):
    """
    use this to control the battle flow. this will call gameflowdirector's  create_sequence_list method to create a sequence list
    when recieve heading, call this commander's execute method
    execute() check who will be next be looking at the sequence list, and call the corresponding publisher (server direct)
    headings and args that leads to this commander:
    1. h:readybattle args:
    2. h:finnishround args: numofoperator
    get identity from self, as we do send Player into MapCommand
    """

    logger = Logger("BattleFlowCommander")
    static_counter:int = 0

    def execute(self) -> str:
        self.__battle_sequence_list = self.get_map().get_game_flow_director().create_sequence_list ( self.get_map().get_attackers(), self.get_map().get_defenders() ) # list of Operator
        chosen_operator:Operator = self.__battle_sequence_list[BattleFlowBattleCommander.static_counter]
        identity:Identity = chosen_operator.get_identity() # do determine which publisher to call
        msg:str = ""

        while not chosen_operator.is_alive():
            BattleFlowBattleCommander.static_counter += 1
            chosen_operator = self.__battle_sequence_list[BattleFlowBattleCommander.static_counter]
            identity = chosen_operator.get_identity()
            msg = f"{BattleFlowBattleCommander.static_counter / 2}_turn"
        
        BattleFlowBattleCommander.static_counter += 1 if BattleFlowBattleCommander.static_counter < len(self.__battle_sequence_list) else 0
        # tenary operator, if true, return 1, else return 0

        if (identity == Identity.ATTACK):
            BattleFlowBattleCommander.logger.info(f"{type(self)}: Attacker's {BattleFlowBattleCommander.static_counter}th operator's turn")
            self.get_map().get_game_data_publisher().publish_client_A_server_actively(msg)
        elif (identity == Identity.DEFEND):
            BattleFlowBattleCommander.logger.info(f"{type(self)}: Defender's {BattleFlowBattleCommander.static_counter}th operator's turn")
            self.get_map().get_game_data_publisher().publish_client_B_server_actively(msg)

        return "success" #success reply incoming message w/ id; use acitve publisher to send who's round to the client

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            BattleFlowBattleCommander.logger.error("battleflowcommander command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 0):
            BattleFlowBattleCommander.logger.error("battleflowcommander command must have no args")
            return "args_len_error"
        
        return None
    
class PlayerShootBattleCommand(MapCommand):
    """
    command: h:shoot args:3 2 => X's 3rd operator shoot at Y's 2nd operator
    """

    logger = Logger("PlayerShootBattleCommand")

    def execute(self) -> str:
        msg:str = f"success_{self.get_args()[0]}_{self.get_args()[1]}_"
        opponent_hp = self.get_map().get_shooting_system().shoot(self.get_map().get_attacker(self.get_args[0]), self.get_map().get_defender(self.get_args[1]))
        msg += str(opponent_hp)
        PlayerCheckSightBattleCommand.logger.debug(f"{type(self)}: {msg}")

        if (self.get_identity == Identity.ATTACK):
            pass
        return msg

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), BattleState):
            PlayerShootBattleCommand.logger.error(f"{type(self)}: shoot command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 2):
            PlayerShootBattleCommand.logger.error(f"{type(self)}: shoot command must have 2 args")
            return "args_len_error"
        else:
            PlayerCheckSightBattleCommand.logger.error(f"{type(self)}: unknown error")
            return "unknown_error"
        return None