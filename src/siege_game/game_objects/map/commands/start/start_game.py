from typing import Callable
from siege_game.game_objects.map.commands.map_command import MapCommand
from siege_game.game_objects.states.state import StartState

class StartGameMapCommand(MapCommand):
    def __init__():
        super().__init__()

    def execute(self) -> bool:
        super().execute()
    
    def check(self) -> bool:
        if not isinstance(self.__map.__game_flow_director.get_state(), StartState):
            return False
        else:
            return True
        
    def content(self) -> None:
        pass
            