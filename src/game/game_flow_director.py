import warnings

class GameFlowDirector:
    __instance = None
    def __init__(self):
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance():
        if GameFlowDirector.__instance == None:
            GameFlowDirector.__instance = GameFlowDirector()
        
        return GameFlowDirector.__instance