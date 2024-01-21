import pytest
from siege_game.game_objects.map_builder import MapBuilder
from siege_game.game_objects.map.map_objects import floor, wall, door, window, soft_wall, entrance, barrier
from siege_game.game_objects.pawn import attacker, defender, operator, shooting_system, sight_checker, shooting_system
from siege_game.game_objects import map_data_processor, game_data_publisher, game_flow_director
from siege_game.game import Game
from collections import deque


testing_game = Game("testid")
testing_map = MapBuilder("map_example", "testid").get_map()

for i in range (0, 5):
    attackers = testing_game.get_map().add_attacker([i, i])
    defenders = testing_game.get_map().add_defender([i, i])

def test_map_data_type():

    assert type(testing_game.get_map().get_map_data()) == dict
    assert type(testing_game.get_map().get_shooting_system()) == shooting_system.ShootingSystem
    assert type(testing_game.get_map().get_sight_checker()) == sight_checker.SightChecker
    assert type(testing_game.get_map().get_map_data_processor()) == map_data_processor.MapDataProcessor
    assert type(testing_game.get_map().get_game_data_publisher()) == game_data_publisher.GameDataPublisher
    assert type(testing_game.get_map().get_game_flow_director()) == game_flow_director.GameFlowDirector
    assert type(testing_game.get_map().get_max_defender_count()) == int
    assert type(testing_game.get_map().get_max_attacker_count()) == int
    assert type(testing_game.get_map().get_attackers()) == deque
    assert type(testing_game.get_map().get_defenders()) == deque
    assert type(testing_game.get_map().get_defender(0)) == defender.Defender
    assert type(testing_game.get_map().get_attacker(0)) == attacker.Attacker

def test_map_data_value():
    assert type((testing_game.get_map()).get_map_data()) == type(testing_map.get_map_data())
    assert type(testing_game.get_map()) == type(testing_map)
    assert type(testing_map.get_map_object([0,0])) == barrier.Barrier
    assert type(testing_map.get_map_object([1,1])) == floor.Floor
    assert type(testing_map.get_map_object([2,2])) == wall.Wall
    assert type(testing_map.get_map_object([3,5])) == door.Door
    assert type(testing_map.get_map_object([3,2])) == window.Window
    assert type(testing_map.get_map_object([4,4])) != soft_wall.SoftWall
    assert type(testing_map.get_map_object([1,3])) == entrance.Entrance
