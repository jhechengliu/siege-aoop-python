#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32

from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
from threading import Thread
import logging
from siege_game.game_objects.logger import Logger


if __name__ == "__main__":
    while True:
        rospy.init_node('cube_position_node', log_level=rospy.DEBUG)
        logging.basicConfig(level=logging.NOTSET)
        logger = Logger("main")
        logger.debug("Start server...")

        game = Game()
        invoker = Invoker.get_instance()

        logger.info("This is info")
        logger.error("This is error")
        logger.debug("This is debug")
        logger.warning("This is warn")
        logger.fatal("What have you done")

        thread1 = Thread(target=game.run, args=())
        thread2 = Thread(target=invoker.run_terminal, args=())

        thread1.start()
        thread2.start()

        thread1.join()
        thread2.join()

        del game
        del invoker
        del thread1
        del thread2
        del logger
        Invoker.remove_instance()

    

