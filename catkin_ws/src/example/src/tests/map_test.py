import pytest
from siege_game.game_objects.map_builder import MapBuilder
from siege_game.game_objects.pawn import attacker, defender, operator, shooting_system, walking_system, sight_checker, shooting_system, walking_system
from siege_game.game_objects import map_data_processor, game_data_publisher, game_flow_director
from siege_game.game import Game
from collections import deque


testing_game = Game()

testing_map = MapBuilder("map_example")
# testing_game.map.get_sight_checker(xxx)
for i in range (0, 5):
    attackers = testing_game.get_map().add_attacker([i, i])
    defenders = testing_game.get_map().add_defender([i, i])

def test_map_data_type():

    assert type(testing_game.get_map().getters().get_map_data()) == dict
    assert type(testing_game.get_map().getters().get_shooting_system()) == shooting_system.ShootingSystem
    assert type(testing_game.get_map().getters().get_walking_system()) == walking_system.WalkingSystem
    assert type(testing_game.get_map().getters().get_sight_checker()) == sight_checker.SightChecker
    assert type(testing_game.get_map().getters().get_map_data_processor()) == map_data_processor.MapDataProcessor
    assert type(testing_game.get_map().getters().get_testing_game_data_publisher()) == game_data_publisher.GameDataPublisher
    assert type(testing_game.get_map().getters().get_game_flow_director()) == game_flow_director.GameFlowDirector
    assert type(testing_game.get_map().getters().get_max_defender_count()) == int
    assert type(testing_game.get_map().getters().get_max_attacker_count()) == int
    assert type(testing_game.get_map().getters().get_attackers()) == deque
    assert type(testing_game.get_map().getters().get_defenders()) == deque
    assert type(testing_game.get_map().getters().get_deffender(0)) == defender.Defender
    assert type(testing_game.get_map().getters().get_attacker(0)) == attacker.Attacker
    assert type(testing_game.get_map().getters().get_map_object([0,0])) == type

def test_map_data_value():
    assert testing_game.get_map().getters().get_map_data() == testing_map.getters().get_map_data()
    assert testing_game.get_map() == testing_map
    assert type(testing_game.get_map()) == type(testing_map)
    assert testing_map.get_map_object([0,0]) == "barrier"
    assert testing_map.get_map_object([1,1]) == "floor"