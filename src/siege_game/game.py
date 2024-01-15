from siege_game.game_objects.map_builder import MapBuilder
import logging
from siege_game.game_objects.player import Player
from siege_game.game_objects.commander import Commander
import time

class Game():
    instance = None
    logger = logging.getLogger("Game")

    def __init__(self):
        self.__defend_player_name:str = "player1"
        self.__attack_player_name:str = "player2"
        self.__map_name = "map_example"
        self.__commander = None
        self.__map = None
        builder = MapBuilder(self.__map_name, self.__defend_player_name, self.__attack_player_name)
        self.__map = builder.get_map()
        self.__commander = Commander(self.__map)
        Game.logger.info(f"Command set: {self.__commander}")

    @classmethod
    def get_instance(cls):
        if (cls.instance == None):
            cls.instance = Game()

        return cls.instance
    
    def run(self):
        self.__map.print_map()
        while (True):
            pass

    def get_commander(self):
        return self.__commander

    def set_defend_player(self, defend_player_name:str):
        self.__defend_player_name = Player(defend_player_name)

    def set_attack_player(self, attack_player_name:str):
        self.__attack_player_name = Player(attack_player_name)




