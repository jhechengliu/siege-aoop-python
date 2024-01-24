import rospy
from std_msgs.msg import String, Int32
# from sensor_msgs.msg import Mouse

from siege_game.game_objects.logger import Logger
from siege_game.game_objects.commander import Commander
import time

class Game():
    """
    The Game class represents a game instance in the Siege game. It handles game state and player interactions.

    Attributes:
        instance: A singleton instance of the Game class.
        logger: A Logger object for logging messages.
        __game_invoker: The Invoker object that manages this game instance.
        __commander: A Commander object that handles game commands.
        __game_id: A unique identifier for the game.
        __attacker_click_count: The number of clicks made by the attacker.
        __defender_click_count: The number of clicks made by the defender.
        ready_count: The number of players ready to start the game.
        __server_client_A_publisher: A rospy.Publisher object for publishing to '/server_client_A_' topic.
        __server_client_B_publisher: A rospy.Publisher object for publishing to '/server_client_B_' topic.
        __server_client_A_message: A String message for the '/server_client_A_' topic.
        __server_client_B_message: A String message for the '/server_client_B_' topic.

    Methods:
        get_attacker_click_count(): Returns the number of clicks made by the attacker.
        get_defender_click_count(): Returns the number of clicks made by the defender.
        increment_attacker_click_count(): Increments the number of clicks made by the attacker.
        increment_defender_click_count(): Increments the number of clicks made by the defender.
        get_client_A_player(): Returns the Player object for client A.
        get_client_B_player(): Returns the Player object for client B.
        get_instance(): Returns the singleton instance of the Game class.
        get_commander(): Returns the Commander object for this game.
        publish_client_A_server_actively(msg): Publishes a message to the '/server_client_A_' topic.
        publish_client_B_server_actively(msg): Publishes a message to the '/server_client_B_' topic.
    """
    instance = None
    logger = Logger("Game")

    def __init__(self, game_id:str, game_invoker):
        """
        Initializes a new instance of the Game class.

        Args:
            game_id (str): The unique identifier for the game.
            game_invoker: The Invoker object that manages this game instance.
        """
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
        """
        Returns the number of clicks made by the attacker.

        Returns:
            int: The number of clicks made by the attacker.
        """
        return self.__attacker_click_count

    def get_defender_click_count(self):
        """
        Returns the number of clicks made by the defender.

        Returns:
            int: The number of clicks made by the defender.
        """
        return self.__defender_click_count
    
    def increment_attacker_click_count(self):
        """
        Increments the number of clicks made by the attacker.
        """
        self.__attacker_click_count += 1

    def increment_defender_click_count(self):
        """
        Increments the number of clicks made by the defender.
        """
        self.__defender_click_count += 1
        
    def get_client_A_player(self):
        """
        Returns the Player object for client A.

        Returns:
            Player: The Player object for client A.
        """
        return self.__game_invoker.get_client_A_player()

    def get_client_B_player(self):
        """
        Returns the Player object for client B.

        Returns:
            Player: The Player object for client B.
        """
        return self.__game_invoker.get_client_B_player()
  
    def get_instance():
        """
        Returns the singleton instance of the Game class.

        Returns:
            Game: The singleton instance of the Game class.
        """
        if Game.instance == None:
            Game.instance = Game()
        return Game.instance

    def get_commander(self):
        """
        Returns the Commander object for this game.

        Returns:
            Commander: The Commander object for this game.
        """
        return self.__commander
    
    
    def publish_client_A_server_actively(self, msg):
        """
        Publishes a message to the '/server_client_A_' topic.

        Args:
            msg: The message to publish.
        """
        self.__server_client_A_message.data = msg
        Game.logger.info(f"Sending active data to server client A channel: {msg}")
        self.__server_client_A_publisher.publish(self.__server_client_A_message)

    def publish_client_B_server_actively(self, msg):
        """
        Publishes a message to the '/server_client_B_' topic.

        Args:
            msg: The message to publish.
        """
        self.__server_client_B_message.data = msg
        Game.logger.info(f"Sending active data to server client B channel: {msg}")
        self.__server_client_B_publisher.publish(self.__server_client_B_message)
    


