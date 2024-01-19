from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.map.commands.battle import Battle
from siege_game.game_objects.logger import Logger

class PlayerMovementBattleCommand(MapCommand):
    """
    command: move 3 3.5 4.5 => Try to move Attacker's 3rd operator to location x=3.5, y=4.5
    """
    logger = Logger("PlayerMovementBattleCommand")

    def execute(self) -> None:
        pass
    
    def check(self) -> bool:
        if isinstance(self.get_map().get_game_flow_director().get_state(), Battle):
            PlayerMovementBattleCommand.logger.error("move command can only be used in battle state")
            return "not_in_battle_state_error"
        elif (len(self.get_args()) != 5):
            PlayerMovementBattleCommand.logger.error("Args len must be 5")
            return "args_len_error"
        elif (self.get_args()[0] != "A" and self.get_args()[0] != "D"):
            PlayerMovementBattleCommand.logger.error("Player type must be Attacker \"A\" or Defender \"D\"")
            return "identity_type_error"

        return None