import warnings

class GameDataPublisher:
    __instance = None
    def __init__(self):
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance(cls):
        if (GameDataPublisher.__instance == None):
            GameDataPublisher.__instance = GameDataPublisher()

        return GameDataPublisher.__instance