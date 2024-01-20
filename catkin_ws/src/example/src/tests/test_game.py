import pytest
from ..siege_game.game import Game

class TestGame():
    def setup_method(self):
        self.game = Game("test_game")
    
    def test_get_commander(self):
        assert self.game.get_commander() == None
    
    def test_get_map(self):
        assert self.game.get_map() == None