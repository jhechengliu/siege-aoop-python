from siege_game.game_objects.logger import Logger
import warnings

class MapDataProcessor:
    logger = Logger("MapDataProcessor")
    """
    This class process the map into two maps for the attacking side and the defending side
    """
    logger = Logger('MapDataProcessor')

    def __init__(self):
        MapDataProcessor.logger.warning("<map_data_processor.py> Use get_instance class method to obtain the instance")
    
    def get_defender_map(self):
        MapDataProcessor.logger.warning("This method is WIP")

    def get_attacker_map(self):
        MapDataProcessor.logger.warning("This method is WIP")
