from siege_game.game_objects.map.commands.map_command import MapCommand
import logging

class PlayerMovementBattleCommand(MapCommand):
    logging = logging.getLogger("PlayerMovementBattleCommand")

    def execute(self) -> None:
        pass
    
    def check(self) -> bool:
        pass
