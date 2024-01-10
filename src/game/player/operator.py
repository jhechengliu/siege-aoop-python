import random
from typing import List
from player import Player

class Operator(Player):
    """
    This class represents an operator in the game.
    It contains the name of the operator and the type of the operator.
    The type of the operator is either Attacker or Defender.

    Attributes:
        xp (int): The experience points of the operator.
        weapons (List[str]): The list of weapons the operator possesses.
        mode (str): The current mode of the operator.
        sight_range (int): The range of vision for the operator.
        sight_distance (int): The maximum distance the operator can see.
        steps (int): The number of steps the operator can take in a turn.
        special_actions (List[str]): The list of special actions the operator can perform.
        available_modes (List[str]): The list of available modes for the operator.
    """
    def __init__(self, player_type: str) -> None:
        super().__init__(player_type)
        self.xp: int = 100
        self.weapons: List[str] = ['gun', 'grenade']
        self.mode: str = 'normal'
        self.sight_range: int = 360
        self.sight_distance: int = 10
        self.steps: int = 5
        self.special_actions: List[str] = ['switch_mode', 'throw_grenade', 'break', 'pre_aim']
        self.available_modes: List[str] = ['normal', 'stealth', 'sniper']

    def aim(self, target: str) -> str:
        """
        Aims at the specified target.

        Args:
            target (str): The target to aim at.

        Returns:
            str: The result of aiming at the target.
        """
        # Implementation logic for aiming at the target
        return f"Aiming at {target}..."

    def shoot(self) -> str:
        """
        Shoots the weapon.

        Returns:
            str: The result of shooting the weapon.
        """
        # Implementation logic for shooting the weapon
        return "Weapon fired!"
    
    def calculate_accuracy(self, distance: int) -> float:
        """
        Calculates the accuracy of the operator's shot based on the distance.

        Args:
            distance (int): The distance to the target.

        Returns:
            float: The accuracy of the shot as a percentage.
        """
        # Implementation logic for calculating accuracy based on distance
        if distance <= self.sight_distance:
            return 100.0
        elif distance <= self.sight_range:
            return 75.0
        else:
            return 50.0

    def calculate_damage(self, accuracy: float) -> int:
        """
        Calculates the damage of the operator's shot based on the accuracy.

        Args:
            accuracy (float): The accuracy of the shot as a percentage.

        Returns:
            int: The damage inflicted by the shot.
        """
        # Implementation logic for calculating damage based on accuracy
        if accuracy >= 90.0:
            return random.randint(80, 100)
        elif accuracy >= 70.0:
            return random.randint(50, 80)
        else:
            return random.randint(20, 50)