from siege_game.game import Game
from siege_game.game_objects.map.commands.setting_up import SetOperatorSettingUpCommand
import pytest
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.commander import Commander

identity = Identity()
game = Game("test_game")
setting_up_attacker = SetOperatorSettingUpCommand(game, ("setoperator", "3", "1.2", "3.4"), Player("testplayer", identity.ATTACK, Commander(game)))
setting_up_defender = SetOperatorSettingUpCommand(game, ("setoperator", "3", "1.2", "3.4"), Player("testplayer", identity.DEFEND, Commander(game)))
for i in range (0, 3):
    setting_up_attacker.get_map().add_attacker([i, i])
    setting_up_defender.get_map().add_defender([i, i])
    # now have 3 attackers and 3 defenders

def test_execute():
    assert setting_up_attacker.execute() == "success_1_left"
    assert setting_up_defender.execute() == "success_1_left"
