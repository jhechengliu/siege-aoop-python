import game.map_builder
import logging
from game.player import Player

logging.basicConfig(level=logging.INFO)

defend_player = Player("DCtime")
attack_player = Player("Hmm")
builder = game.map_builder.MapBuilder.get_instance("map_example", defend_player, attack_player)
map = builder.get_map()
map.print_map()
