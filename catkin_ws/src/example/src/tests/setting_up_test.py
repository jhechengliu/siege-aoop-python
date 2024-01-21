from siege_game.game import Game
from siege_game.game_objects.map.commands.setting_up import SetOperatorSettingUpCommand
import pytest
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.commander import Commander

identity = Identity()
game = Game("test_game")
game.get_map().__max_attacker_count = 5
game.get_map().__max_defender_count = 5
setting_up_attacker = SetOperatorSettingUpCommand(game, ("1.2", "3.4"), Player("testplayer", identity.ATTACK, Commander(game)))
setting_up_defender = SetOperatorSettingUpCommand(game, ("1.2", "3.4"), Player("testplayer", identity.DEFEND, Commander(game)))
for i in range (0, 3):
    setting_up_attacker.get_map().add_attacker([i, i])
    setting_up_defender.get_map().add_defender([i, i])
    # now have 3 attackers and 3 defenders

def test_check():
    assert setting_up_attacker.check() == None
    assert setting_up_defender.check() == None

def test_execute():
    assert setting_up_attacker.execute() == "success_1_left"
    assert setting_up_defender.execute() == "success_1_left"

def test_check_if_ready_to_next_stage():
    pass
