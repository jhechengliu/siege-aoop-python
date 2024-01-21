import pytest
from siege_game.game_objects.pawn.sight_checker import SightChecker
from siege_game.game_objects.map_builder import MapBuilder
from siege_game.game import Game
from collections import deque

testing_map = MapBuilder("map_example", "testid")

def test_sight_checker():
    sight_checker = SightChecker()
    map_data = (testing_map.get_map()).get_map_data()

    assert type(map_data) == dict
    assert sight_checker.check_sight(map_data, [0,0], [0,0]) == False
    assert sight_checker.check_sight(map_data, [1,1], [1,2]) == True
    assert sight_checker.check_sight(map_data, [1,1], [8,8]) == False
    assert sight_checker.check_sight(map_data, [1.5,1.7], [3.3, 6.6]) == False
    assert sight_checker.check_sight(map_data, [0,0], [0,1]) == False
