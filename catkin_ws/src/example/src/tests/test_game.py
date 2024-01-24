import sys
import os
import unittest
import shortuuid
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from siege_game.game import Game
from siege_game.game_objects.game_invoker import GameInvoker
from siege_game.game_objects.constants.identity import Identity
class TestGame(unittest.TestCase):
    def setUp(self):
        self.__new_id = shortuuid.uuid()
        self.__game_invoker = GameInvoker(self.__new_id)
        self.__game = Game(self.__new_id, self.__game_invoker)
        self.__game_invoker.make_client_A_player("Alice", Identity.ATTACK, self.__game)
        

    def test_get_client_A_player(self):
        self.assertEqual(self.__game.get_client_A_player().get_name(), "Alice")
        self.assertEqual(self.__game.get_client_A_player().get_identity(), Identity.ATTACK)

        self.__game.increment_attacker_click_count()
        self.assertEqual(self.__game.get_attacker_click_count(), 1)

if __name__ == '__main__':
    unittest.main()