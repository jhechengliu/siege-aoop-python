from siege_game.game import Game
import pytest

game = Game("test_game")
setting_up = game.get_setting_up()

def test_execute():
    # Create a mock instance of the SettingUp class

    # Set the number of attackers and defenders in the map
    setting_up.get_map().set_attackers([1, 2, 3])
    setting_up.get_map().set_defenders([1, 2, 3, 4])

    # Call the execute method
    result = setting_up.execute()

    # Check if the result is as expected
    assert result == "success_4_left"