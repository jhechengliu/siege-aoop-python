import logging
import warnings

class MapDataProcessor:
    """
    This class process the map into two maps for the attacking side and the defending side
    """
    __instance = None
    logger = logging.getLogger('MapDataProcessor')

    def __init__(self):
        MapDataProcessor.logger.warning("<map_data_processor.py> Use get_instance class method to obtain the instance")

    @classmethod
    def get_instance(cls):
        if MapDataProcessor.__instance == None:
            MapDataProcessor.__instance = MapDataProcessor()
        
        return MapDataProcessor.__instance
    
    def get_defender_map(self):
        warnings.warn("This method is WIP")

    def get_attacker_map(self):
        warnings.warn("This method is WIP")
