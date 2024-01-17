import rospy
from std_msgs.msg import String, Int32
# from sensor_msgs.msg import Mouse

from siege_game.game_objects.map_builder import MapBuilder
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.commander import Commander
import time

class MySubscriberListener(rospy.SubscribeListener):
    def __init__(self):
        super(MySubscriberListener, self).__init__()

    def peer_unsubscribe(self, topic_name, num_peers):
        print("BRUH")

class Game():
    instance = None
    logger = Logger("Game")

    def __init__(self):
        self.__map_name = "map_example"
        self.__commander = None
        self.__map = None
        builder = MapBuilder(self.__map_name)
        self.__map = builder.get_map()
        self.__commander = Commander(self)
        Game.logger.info(f"Command set: {self.__commander}")

        self.__my_listener = MySubscriberListener()
        self.__my_publisher = rospy.Publisher(
            name='~test',
            data_class=Int32,
            subscriber_listener=self.__my_listener,
            queue_size=10)
        
        self.__int32data = Int32()

    @classmethod
    def get_instance(cls):
        if (cls.instance == None):
            cls.instance = Game()

        return cls.instance
    
    def run(self):
        counter = 0
        self.__int32data.data = counter
        self.__map.print_map()
        while not rospy.is_shutdown():
            self.__my_publisher.publish(self.__int32data)
            rospy.sleep(0.1)
            counter += 1

    def get_commander(self):
        return self.__commander
    
    def get_map(self):
        return self.__map




