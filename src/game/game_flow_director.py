import logging

class GameFlowDirector:
    __instance = None
    logger = logging.getLogger('GameFlowDirector')
    def __init__(self):
        GameFlowDirector.logger.warning("<game_flow_director.py> Use get_instance class method to obtain the instance")

    @classmethod
    def get_instance(cls):
        if GameFlowDirector.__instance == None:
            GameFlowDirector.__instance = GameFlowDirector()
        
        return GameFlowDirector.__instance