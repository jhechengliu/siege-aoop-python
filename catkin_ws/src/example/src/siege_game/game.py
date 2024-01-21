import rospy
from std_msgs.msg import String, Int32
# from sensor_msgs.msg import Mouse

from siege_game.game_objects.map_builder import MapBuilder
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.commander import Commander
import time

class Game():
    instance = None
    logger = Logger("Game")

    def __init__(self, game_id:str, game_invoker):
        self.__game_invoker = game_invoker
        self.__map_name = "map_example"
        self.__commander = None
        self.__map = None
        self.__game_id = game_id
        builder = MapBuilder(self.__map_name, self.__game_id)
        self.__map = builder.get_map()
        self.__commander = Commander(self)
        Game.logger.info(f"Command set: {self.__commander}")
        
        self.__attacker_click_count = 0
        self.__defender_click_count = 0
        self.ready_count = 0

    def get_attacker_click_count(self):
        return self.__attacker_click_count

    def get_defender_click_count(self):
        return self.__defender_click_count
    
    def increment_attacker_click_count(self):
        self.__attacker_click_count += 1

    def increment_defender_click_count(self):
        self.__defender_click_count += 1
        
    def get_client_A_player(self):
        return self.__game_invoker.get_client_A_player()

    def get_client_B_player(self):
        return self.__game_invoker.get_client_B_player()
  
    def get_instance():
        if Game.instance == None:
            Game.instance = Game()
        return Game.instance

    def get_commander(self):
        return self.__commander
    
    def get_map(self):
        return self.__map


