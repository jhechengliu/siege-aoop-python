from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
from threading import Thread
import logging

logging.basicConfig(level=logging.NOTSET)

game = Game.get_instance()
invoker = Invoker(game)

thread1 = Thread(target=game.run, args=())
thread2 = Thread(target=invoker.run_terminal, args=())

thread1.start()
thread2.start()


thread1.join()
thread2.join()

