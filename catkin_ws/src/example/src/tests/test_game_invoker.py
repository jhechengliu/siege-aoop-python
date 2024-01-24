import unittest
from src.siege_game.game_objects.game_invoker import GameInvoker
from src.siege_game.game_objects.player import Player

class TestGameInvoker(unittest.TestCase):
    def setUp(self):
        self.game_invoker = GameInvoker()

    def test_get_client_A_player(self):
        player_A = Player("Alice", "A", "CommanderA")
        self.game_invoker.set_client_A_player(player_A)
        self.assertEqual(self.game_invoker.get_client_A_player(), player_A)

if __name__ == '__main__':
    unittest.main()