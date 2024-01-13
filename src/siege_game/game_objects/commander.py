from siege_game.game_objects.map.map import Map

class Commander():
    def __init__(self, map:Map):
        self.__map = map
        self.__command_types = {

        }

    def execute_command(self, command:str):
        pass