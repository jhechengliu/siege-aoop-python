from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import SettingUpState
import logging

class InitPlayerSettingUpCommand(MapCommand):
    """
    command: init A 1 2 => Init Attacker at location x=1, y=2
    """
    logger = logging.getLogger("PutPlayerSettingUpCommand")

    def execute(self) -> None:
        operator_type = self.get_args()[0]
        x = float(self.get_args()[1])
        y = float(self.get_args()[2])

        if (operator_type == "A"):
            self.get_map().add_attacker([x, y])
        elif (operator_type == "D"):
            self.get_map().add_defender([x, y])

        self.get_map().map_status()

    def check(self) -> bool:
        if isinstance(self.get_map().get_game_flow_director().get_state(), SettingUpState):
            InitPlayerSettingUpCommand.logger.error("Init command can only be used in setting up state")
            return False
        elif (len(self.get_args()) != 3):
            InitPlayerSettingUpCommand.logger.error("Args len must be 3")
            return False
        elif (self.get_args()[0] != "A" and self.get_args()[0] != "D"):
            InitPlayerSettingUpCommand.logger.error("Player type must be Attacker \"A\" or Defender \"D\"")
            return False
        elif (self.get_args()[0] == "A" and len(self.get_map().get_attackers()) >= self.get_map().get_max_attacker_count()):
            InitPlayerSettingUpCommand.logger.error("Cant add another Attacker because attacker count reached max limit")
            return False
        elif (self.get_args()[0] == "D" and len(self.get_map().get_defenders()) >= self.get_map().get_max_defender_count()):
            InitPlayerSettingUpCommand.logger.error("Cant add another Defender because defender count reached max limit")
            return False
        
        x = self.get_args()[1]
        y = self.get_args()[2]

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
        
    
