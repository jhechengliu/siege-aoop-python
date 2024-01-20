from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.battle import Battle
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.pawn.operator import Operator
from siege_game.game_objects.constants.identity import Identity

class PlayerMovementBattleCommand(MapCommand):
    """
    command: move 3 3.5 4.5 => Try to move Attacker's 3rd operator to location x=3.5, y=4.5
    """
    logger = Logger("PlayerMovementBattleCommand")

    def execute(self) -> None:
        # calulate if movable, true/false
        if (self.get_send_player().get_identity() == Identity.ATTACK):
            operator:Operator = self.get_map().get_attacker(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            # check_sight(map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
            if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), [self.get_args[1](), self.get_args[2]()]) ):
                self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
                return "success"
            else:
                return "not-in-sight-error"
                # use dash (-) to connect heading

        elif (self.get_send_player().get_identity() == Identity.DEFEND):
            operator:Operator = self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])
            if (self.get_map().get_sight_checker().check_sight(self.get_map().get_map_data(), operator.get_location(), [self.get_args[1](), self.get_args[2]()]) ):
                self.get_map().get_defender(self.get_args[0]).set_location([self.get_args[1], self.get_args[2]])   
                return "success"
            else:
                return "not-in-sight-error" 
    
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
    will be call whan:
    client move successfully, send a message to server
    """
    logger = Logger("PlayerCheckSightCommand")

    def execute(self) -> None:
        pass

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), Battle):
            PlayerCheckSightCommand.logger.error("check_sight command can only be used in battle state")
            return "not_in_battle_state_error"

        return None