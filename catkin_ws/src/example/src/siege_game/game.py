import rospy
from std_msgs.msg import String, Int32
# from sensor_msgs.msg import Mouse

from siege_game.game_objects.logger import Logger
from siege_game.game_objects.commander import Commander
import time

class Game():
    instance = None
    logger = Logger("Game")

    def __init__(self, game_id:str, game_invoker):
        self.__game_invoker = game_invoker
        self.__commander = None
        self.__game_id = game_id
        self.__commander = Commander(self)
        Game.logger.info(f"Command set: {self.__commander}")
        
        self.__attacker_click_count = 0
        self.__defender_click_count = 0
        self.ready_count = 0

        
        self.__server_client_A_publisher = rospy.Publisher('/server_client_A_' + self.__game_id, String, queue_size=10)
        self.__server_client_B_publisher = rospy.Publisher('/server_client_B_' + self.__game_id, String, queue_size=10)

        self.__server_client_A_message = String()
        self.__server_client_B_message = String()

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
    
    
    def publish_client_A_server_actively(self, msg):
        self.__server_client_A_message.data = msg
        Game.logger.info(f"Sending active data to server client A channel: {msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)

    def publish_client_B_server_actively(self, msg):
        self.__server_client_B_message.data = msg
        Game.logger.info(f"Sending active data to server client B channel: {msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)
    


