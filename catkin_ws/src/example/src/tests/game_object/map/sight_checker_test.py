import pytest
from siege_game.game_objects.pawn.sight_checker import SightChecker
from siege_game.game import Game

def test_sight_checker():
    game = Game()

    # game.map.get_sight_checker(xxx)