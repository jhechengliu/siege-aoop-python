from game import Player

class Operator(Player):
    """
    This class represents an operator in the game.
    It contains the name of the operator and the type of the operator.
    The type of the operator is either Attacker or Defender.

    Attributes:
        xp (int): The experience points of the operator.
        weapons (list): The list of weapons the operator possesses.
        mode (str): The current mode of the operator.
        sight_range (int): The range of vision for the operator.
        sight_distance (int): The maximum distance the operator can see.
        steps (int): The number of steps the operator can take in a turn.
        special_actions (list): The list of special actions the operator can perform.
        available_modes (list): The list of available modes for the operator.
    """
    def __init__(self, player_type):
        super().__init__(player_type)
        self.xp = 100
        self.weapons = ['gun', 'grenade']
        self.mode = 'normal'
        self.sight_range = 360
        self.sight_distance = 10
        self.steps = 5
        self.special_actions = ['switch_mode', 'throw_grenade', 'break', 'pre_aim']
        self.available_modes = ['normal', 'stealth', 'sniper']
