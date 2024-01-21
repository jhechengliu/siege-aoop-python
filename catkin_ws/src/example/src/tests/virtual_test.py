from siege_game.game_objects.game_invoker import GameInvoker

game_id = "test"
game_invoker = GameInvoker("test")
game_invoker.client_A_callback("test sign A ATT")   # create player for client A
game_invoker.client_B_callback("test sign D DEF")   # create player for client B
game = game_invoker.get_game()
assert isinstance(game_invoker, GameInvoker)

from siege_game.game_objects.map.map import Map
from siege_game.game_objects.map_builder import MapBuilder

map_builder = MapBuilder("example", game_id)
map = map_builder.get_map() # map generated

assert isinstance(map, Map)
assert isinstance(map.get_map_data(), dict)
assert game_invoker.client_A_callback("id setoperator 0 1 2") == "id success_1_left"
assert game_invoker.client_A_callback("id setoperator 1 1 2") == "id success_0_right"
assert game_invoker.client_B_callback("id setoperator 0 1 2") == "id success_1_left"
assert game_invoker.client_B_callback("id setoperator 1 1 2") == "id success_0_right"