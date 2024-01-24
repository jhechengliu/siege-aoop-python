import sys
import os
import unittest
import shortuuid
sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), '../../src')))

from siege_game.game_objects.game_invoker import GameInvoker
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.commander import Commander
from siege_game.game import Game

class TestGameInvoker(unittest.TestCase):
    def setUp(self):
        self.game_invoker = GameInvoker(shortuuid.uuid())

    def test_get_client_A_player(self):
        new_uuid = shortuuid.uuid()
        player_A = Player("Alice", Identity.ATTACK, Commander(Game(shortuuid.uuid(new_uuid), GameInvoker(new_uuid))))
        self.assertEqual(self.game_invoker.get_client_A_player(), player_A)

if __name__ == '__main__':
    unittest.main()