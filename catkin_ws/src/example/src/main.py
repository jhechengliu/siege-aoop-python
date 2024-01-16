#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
from threading import Thread
import logging

def fix_logging(level=logging.WARNING):
    console = logging.StreamHandler()
    console.setLevel(level)
    logging.getLogger('').addHandler(console)
    formatter = logging.Formatter('%(levelname)-8s:%(name)-12s: %(message)s')
    console.setFormatter(formatter)
    logging.getLogger('').addHandler(console)

if __name__ == "__main__":

    logging.basicConfig(level=logging.NOTSET)

    game = Game.get_instance()
    invoker = Invoker(game)
    rospy.init_node('cube_position_node')
    fix_logging()

    thread1 = Thread(target=game.run, args=())
    thread2 = Thread(target=invoker.run_terminal, args=())

    thread1.start()
    thread2.start()


    thread1.join()
    thread2.join()

    

