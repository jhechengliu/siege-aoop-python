import warnings

class MapDataProcessor:
    __instance = None
    def __init__(self):
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance():
        if MapDataProcessor.__instance == None:
            MapDataProcessor.__instance = MapDataProcessor()
        
        return MapDataProcessor.__instance
