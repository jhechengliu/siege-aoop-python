import warnings

class MapDataProcessor:
    """
    This class process the map into two maps for the attacking side and the defending side
    """
    __instance = None
    def __init__(self):
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance(cls):
        if MapDataProcessor.__instance == None:
            MapDataProcessor.__instance = MapDataProcessor()
        
        return MapDataProcessor.__instance
