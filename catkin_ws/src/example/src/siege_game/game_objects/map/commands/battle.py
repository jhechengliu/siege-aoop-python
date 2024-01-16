from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.battle import Battle
from siege_game.game_objects.logger import Logger

class PlayerMovementBattleCommand(MapCommand):
    """
    command: move A 1.5 2.5 3.5 4.5 => Move Attacker at location x=1.5, y=2.5 to location x=3.5, y=4.5
    """
    logger = Logger("PlayerMovementBattleCommand")

    def execute(self) -> None:
        pass
    
    def check(self) -> bool:
        if isinstance(self.get_map().get_game_flow_director().get_state(), Battle):
            PlayerMovementBattleCommand.logger.error("move command can only be used in battle state")
            return False
        elif (len(self.get_args()) != 5):
            PlayerMovementBattleCommand.logger.error("Args len must be 5")
            return False
        elif (self.get_args()[0] != "A" and self.get_args()[0] != "D"):
            PlayerMovementBattleCommand.logger.error("Player type must be Attacker \"A\" or Defender \"D\"")
            return False

        return True