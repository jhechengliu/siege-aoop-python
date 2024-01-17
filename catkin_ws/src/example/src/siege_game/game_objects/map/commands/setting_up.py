from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import SettingUpState
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.invoker import Invoker

class SetOperatorSettingUpCommand(MapCommand):
    """
    command: setoperator 1.5 2.5 => Set an operator of yours at location x=1.5, y=2.5
    """
    logger = Logger("SetOperatorSettingUpCommand")

    def execute(self) -> None:
        operator_type = self.get_identity()
        x = float(self.get_args()[0])
        y = float(self.get_args()[1])

        if (operator_type == Identity.ATTACK):
            self.get_map().add_attacker([x, y])
        elif (operator_type == Identity.DEFEND):
            self.get_map().add_defender([x, y])

        self.get_map().map_status()

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), SettingUpState):
            SetOperatorSettingUpCommand.logger.error(f"setoperator command can only be used in setting up state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return "not_in_setting_up_state_error"
        elif (len(self.get_args()) != 2):
            SetOperatorSettingUpCommand.logger.error("Args len must be 3")
            return "args_len_error"
        elif (self.get_identity() == Identity.ATTACK and len(self.get_map().get_attackers()) >= self.get_map().get_max_attacker_count()):
            SetOperatorSettingUpCommand.logger.error("Cant add another Attacker because attacker count reached max limit")
            return "attacker_reach_max_limit_error"
        elif (self.get_identity() == Identity.DEFEND and len(self.get_map().get_defenders()) >= self.get_map().get_max_defender_count()):
            SetOperatorSettingUpCommand.logger.error("Cant add another Defender because defender count reached max limit")
            return "defender_reach_max_limit_error"
        
        x = self.get_args()[0]
        y = self.get_args()[1]

        try:
            float(x)
        except ValueError:
            SetOperatorSettingUpCommand.logger.error("x must be a number")
            return "x_not_number_error"
        
        try:
            float(y)
        except ValueError:
            SetOperatorSettingUpCommand.logger.error("y must be a number")
            return "y_not_number_error"
        
        return None
        
class FinishSettingUpCommand(MapCommand):
    """
    command: finishsettingup => Finish setting up
    """
    logger = Logger("FinishSettingUpCommand")

    def execute(self) -> None:
        self.get_send_player().set_has_finish_setting_up(True)

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), SettingUpState):
            FinishSettingUpCommand.logger.error(f"Game isn't in start state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return "not_in_setting_up_state_error"
        elif (len(self.get_args()) != 0):
            FinishSettingUpCommand.logger.error("start command must have no args")
            return "args_len_error"
        elif (self.get_identity() == Identity.ATTACK and len(self.get_map().get_attackers()) != self.get_map().get_max_attacker_count()):
            FinishSettingUpCommand.logger.error("Attacker count must be equal to max attacker count")
            return "attacker_not_equal_max_count"
        elif (self.get_identity() == Identity.DEFEND and len(self.get_map().get_defenders()) != self.get_map().get_max_defender_count()):
            FinishSettingUpCommand.logger.error("Defender count must be equal to max defender count")
            return "defender_not_equal_max_count"
        else:
            return None
        

class StartBattleSettingUpCommand(MapCommand):
    """
    command: startbattle => start the battle 
    """
    logger = Logger("StartBattleSettingUp")

    def execute(self) -> None:
        self.get_map().get_game_flow_director().next_state()
        StartBattleSettingUpCommand.logger.info(f"Battle starts! Current State: {self.get_map().get_game_flow_director().get_state()}")

    def check(self) -> bool:
        if (len(self.get_args()) != 0):
            StartBattleSettingUpCommand.logger.error("startbattle command args must be 0")
            return "args_len_error"
        
        elif (Invoker.get_instance().get_server_player() != None and not Invoker.get_instance().get_server_player().get_has_finish_setting_up()):
            StartBattleSettingUpCommand.logger.error("Server player isnt ready")
            return "server_player_not_ready"
        
        return None
        