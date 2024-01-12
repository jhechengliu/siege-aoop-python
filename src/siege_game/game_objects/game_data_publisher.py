import logging
import warnings
from siege_game.game_objects.map.map import Map  
class GameDataPublisher:
    logger = logging.getLogger('GameDataPublisher')
    __instance = None
    def __init__(self):
        GameDataPublisher.logger.warning("<game_data_publisher.py> Use get_instance class method to obtain the instance")

    @classmethod
    def get_instance(cls):
        if (GameDataPublisher.__instance == None):
            GameDataPublisher.__instance = GameDataPublisher()

        return GameDataPublisher.__instance
    
    def publish(self):
        warnings.warn("This method is WIP or broken")