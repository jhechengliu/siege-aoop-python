from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import SettingUpState
from siege_game.game_objects.constants.identity import Identity
import logging
from siege_game.game_objects.invoker import Invoker

class InitPlayerSettingUpCommand(MapCommand):
    """
    command: setoperator 1.5 2.5 => Set an operator of yours at location x=1.5, y=2.5
    """
    logger = logging.getLogger("PutPlayerSettingUpCommand")

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
            InitPlayerSettingUpCommand.logger.error(f"setoperator command can only be used in setting up state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return False
        elif (len(self.get_args()) != 2):
            InitPlayerSettingUpCommand.logger.error("Args len must be 3")
            return False
        elif (self.get_identity() == Identity.ATTACK and len(self.get_map().get_attackers()) >= self.get_map().get_max_attacker_count()):
            InitPlayerSettingUpCommand.logger.error("Cant add another Attacker because attacker count reached max limit")
            return False
        elif (self.get_identity() == Identity.DEFEND and len(self.get_map().get_defenders()) >= self.get_map().get_max_defender_count()):
            InitPlayerSettingUpCommand.logger.error("Cant add another Defender because defender count reached max limit")
            return False
        
        x = self.get_args()[0]
        y = self.get_args()[1]

        try:
            float(x)
        except ValueError:
            InitPlayerSettingUpCommand.logger.error("x must be a number")
            return False
        
        try:
            float(y)
        except ValueError:
            InitPlayerSettingUpCommand.logger.error("y must be a number")
            return False
        
        return True
        
class FinishSettingUpCommand(MapCommand):
    """
    command: finishsettingup => Finish setting up
    """
    logger = logging.getLogger("FinishSettingUpCommand")

    def execute(self) -> None:
        pass

    def check(self) -> bool:
        if not isinstance(self.get_map().get_game_flow_director().get_state(), SettingUpState):
            FinishSettingUpCommand.logger.error(f"Game isn't in start state. Current State: {self.get_map().get_game_flow_director().get_state()}")
            return False
        elif (len(self.get_args()) != 0):
            FinishSettingUpCommand.logger.error("start command must have no args")
            return False
        elif (self.get_identity() == Identity.ATTACK and len(self.get_map().get_attackers()) != self.get_map().get_max_attacker_count()):
            FinishSettingUpCommand.logger.error("Attacker count must be equal to max attacker count")
            return False
        elif (self.get_identity() == Identity.DEFEND and len(self.get_map().get_defenders()) != self.get_map().get_max_defender_count()):
            FinishSettingUpCommand.logger.error("Defender count must be equal to max defender count")
            return False
        else:
            return True
        

class StartBattleSettingUpCommand(MapCommand):
    """
    command startbattle => start the battle 
    """
    logger = logging.getLogger("StartBattleSettingUp")

    def execute(self) -> None:
        self.get_map().get_game_flow_director().next_state()

    def check(self) -> bool:
        if (len(self.get_args()) != 0):
            StartBattleSettingUpCommand.logger.error("startbattle command args must be 0")
            return False
        
        elif (Invoker.get_instance().get_server_player() != None and not Invoker.get_instance().get_server_player().get_has_finish_setting_up()):
            StartBattleSettingUpCommand.logger.error("Server player isnt ready")
            return False
        
        return True
        