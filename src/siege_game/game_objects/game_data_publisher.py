import logging
import warnings
class GameDataPublisher:
    logger = logging.getLogger('GameDataPublisher')

    def __init__(self):
        GameDataPublisher.logger.warning("<game_data_publisher.py> Use get_instance class method to obtain the instance")
    
    def publish(self):
        warnings.warn("This method is WIP or broken")