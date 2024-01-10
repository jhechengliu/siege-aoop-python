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
        sight_angle (int): The angle of vision for the operator.
        sight_direction (int): The direction the operator is facing.
        steps (int): The number of steps the operator can take in a turn.
        special_actions (List[str]): The list of special actions the operator can perform.
        available_modes (List[str]): The list of available modes for the operator.

    Methods:
        get_xp() -> int: Returns the experience points of the operator.
        get_weapons() -> List[str]: Returns the list of weapons the operator possesses.
        get_mode() -> str: Returns the current mode of the operator.
        get_sight_range() -> int: Returns the range of vision for the operator.
        get_sight_distance() -> int: Returns the maximum distance the operator can see.
        get_sight_angle() -> int: Returns the angle of vision for the operator.
        get_sight_direction() -> int: Returns the direction the operator is facing.
        get_steps() -> int: Returns the number of steps the operator can take in a turn.
        set_xp(xp: int): Sets the experience points of the operator.
        set_mode(mode: str): Sets the current mode of the operator.
        set_sight_range(sight_range: int): Sets the range of vision for the operator.
        set_sight_distance(sight_distance: int): Sets the maximum distance the operator can see.
        set_sight_angle(sight_angle: int): Sets the angle of vision for the operator.
        set_sight_direction(sight_direction: int): Sets the direction the operator is facing.
        set_steps(steps: int): Sets the number of steps the operator can take in a turn.
        switch_mode(mode: str): Switches the operator's mode.
        throw_grenade() -> str: Throws a grenade.
        break() -> str: Breaks door/window/soft wall.
        aim(target: str) -> str: Aims at the specified target.
        shoot() -> str: Shoots the weapon.
        calculate_accuracy(distance: int) -> float: Calculates the accuracy of the operator's shot based on the distance.
        calculate_damage(accuracy: float) -> int: Calculates the damage of the operator's shot based on the accuracy.
    """
    def __init__(self, player_type: str) -> None:
        super().__init__(player_type)
        self.xp: int = 100
        self.weapons: List[str] = ['gun', 'grenade']
        self.mode: str = 'normal'
        self.sight_range: int = 360
        self.sight_distance: int = 10
        self.sight_angle: int = 360
        self.sight_direction: int = 0
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