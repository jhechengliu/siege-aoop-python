from siege_game.game import Game
from siege_game.game_objects.map.commands.setting_up import SetOperatorSettingUpCommand
import pytest
from siege_game.game_objects.player import Player
from siege_game.game_objects.constants.identity import Identity
from siege_game.game_objects.commander import Commander

game = Game("test_game")
setting_up = SetOperatorSettingUpCommand(game, ("setoperator", "0", "1.2", "3.4"), Player("testplayer", Identity.ATTACKER, Commander(game)))

def test_execute():
    # Create a mock instance of the SettingUp class

    # Set the number of attackers and defenders in the map
    setting_up.get_map().set_attackers([1, 2, 3])
    setting_up.get_map().set_defenders([1, 2, 3, 4])

    # Call the execute method
    result = setting_up.execute()

    # Check if the result is as expected
    assert result == "success_4_left"