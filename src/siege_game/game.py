from siege_game.game_objects.map_builder import MapBuilder
import logging
from siege_game.game_objects.player import Player
from siege_game.game_objects.commander import Commander
import time

class Game():
    instance = None
    logger = logging.getLogger("Game")

    def __init__(self, defend_player_name:str = "Player1", attack_player_name:str = "Player2"):
        self.__defend_player_name:str = defend_player_name
        self.__attack_player_name:str = attack_player_name
        self.__map_name = "map_example"
        self.__commander = None
        self.__map = None

        defend_player = Player(self.__defend_player_name)
        attack_player = Player(self.__attack_player_name)
        builder = MapBuilder(self.__map_name, defend_player, attack_player)
        self.__map = builder.get_map()
        self.__commander = Commander(self.__map)
        Game.logger.info(f"Command set: {self.__commander}")

    @classmethod
    def get_instance(cls, defend_player_name:str, attack_player_name:str):
        if (cls.instance == None):
            cls.instance = Game(defend_player_name, attack_player_name)

        return cls.instance
    
    def run(self):
        self.__map.print_map()
        while (True):
            pass

    def get_commander(self):
        return self.__commander






