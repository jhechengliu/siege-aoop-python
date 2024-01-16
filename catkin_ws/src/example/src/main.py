#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
from threading import Thread
import logging

class ConnectPythonLoggingToROS(logging.Handler):

    MAP = {
        logging.DEBUG:rospy.logdebug,
        logging.INFO:rospy.loginfo,
        logging.WARNING:rospy.logwarn,
        logging.ERROR:rospy.logerr,
        logging.CRITICAL:rospy.logfatal
    }

    def emit(self, record):
        try:
            self.MAP[record.levelno]("%s: %s" % (record.name, record.msg))
        except KeyError:
            rospy.logerr("unknown log level %s LOG: %s: %s" % (record.levelno, record.name, record.msg))


if __name__ == "__main__":

    logging.basicConfig(level=logging.NOTSET)

    game = Game.get_instance()
    invoker = Invoker(game)
    #reconnect logging calls which are children of this to the ros log system
    logging.getLogger('trigger').addHandler(ConnectPythonLoggingToROS())
    #logs sent to children of trigger with a level >= this will be redirected to ROS
    logging.getLogger('trigger').setLevel(logging.DEBUG)

    rospy.init_node('triggerbox_host', log_level=rospy.DEBUG)
    # rospy.init_node('cube_position_node')

    thread1 = Thread(target=game.run, args=())
    thread2 = Thread(target=invoker.run_terminal, args=())

    thread1.start()
    thread2.start()


    thread1.join()
    thread2.join()

    

