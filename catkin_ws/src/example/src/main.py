#!/usr/bin/env python3
import rospy
from std_msgs.msg import String, Int32

from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
from threading import Thread
import logging
from siege_game.game_objects.logger import Logger


if __name__ == "__main__":
    rospy.init_node('cube_position_node', log_level=rospy.DEBUG)
    logging.basicConfig(level=logging.NOTSET)
    logger = Logger("main")

    game = Game.get_instance()
    invoker = Invoker(game)

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

    

