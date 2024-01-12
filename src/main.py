from siege_game.game_objects.map_builder import MapBuilder
import logging
from siege_game.game_objects.player import Player

logging.basicConfig(level=logging.INFO)

defend_player = Player("DCtime")
attack_player = Player("JL")
builder = MapBuilder.get_instance("map_example", defend_player, attack_player)
map = builder.get_map()
map.print_map()
