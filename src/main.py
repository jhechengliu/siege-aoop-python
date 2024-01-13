from siege_game.game import Game
from siege_game.game_objects.invoker import Invoker
game = Game.get_instance("DCtime", "JL")
invoker = Invoker()
game.run()

