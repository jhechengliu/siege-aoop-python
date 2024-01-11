import random
from typing import List
from player import Player

class Operator(Player):
    """
    This class represents an operator in the game.
    It contains the name of the operator and the type of the operator.
    The type of the operator is either Attacker or Defender.

    Attributes:
        hp (int): The health points of the operator.
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
        get_hp() -> int: Returns the health points of the operator.
        get_weapons() -> List[str]: Returns the list of weapons the operator possesses.
        get_mode() -> str: Returns the current mode of the operator.
        get_sight_range() -> int: Returns the range of vision for the operator.
        get_sight_distance() -> int: Returns the maximum distance the operator can see.
        get_sight_angle() -> int: Returns the angle of vision for the operator.
        get_sight_direction() -> int: Returns the direction the operator is facing.
        get_steps() -> int: Returns the number of steps the operator can take in a turn.
        set_hp(hp: int): Sets the health points of the operator.
        set_mode(mode: str): Sets the current mode of the operator.
        set_sight_range(sight_range: int): Sets the range of vision for the operator.
        set_sight_distance(sight_distance: int): Sets the maximum distance the operator can see.
        set_sight_angle(sight_angle: int): Sets the angle of vision for the operator.
        set_sight_direction(sight_direction: int): Sets the direction the operator is facing.
        set_steps(steps: int): Sets the number of steps the operator can take in a turn.
        switch_mode(mode: str): Switches the operator's mode.
        ##break() -> str: Breaks door/window/soft wall. ##Use breakable object class?
    """
    def __init__(self, player_type: str) -> None:
        super().__init__(player_type)
        self.hp: int = 100
        self.weapons: List[str] = ['gun', 'grenade']
        self.mode: str = 'normal'
        self.sight_range: int = 360
        self.sight_distance: int = 10
        self.sight_angle: int = 360
        self.sight_direction: int = 0
        self.steps: int = 5
        self.special_actions: List[str] = ['switch_mode', 'throw_grenade', 'break', 'pre_aim']
        self.available_modes: List[str] = ['normal', 'stealth', 'sniper']

    def get_hp(self) -> int:
        """
        Returns the health points of the operator.

        Returns:
            int: The health points of the operator.
        """
        return self.hp

    def get_weapons(self) -> List[str]:
        """
        Returns the list of weapons the operator possesses.

        Returns:
            List[str]: The list of weapons the operator possesses.
        """
        return self.weapons
    
    def get_mode(self) -> str:
        """
        Returns the current mode of the operator.

        Returns:
            str: The current mode of the operator.
        """
        return self.mode
    
    def get_sight_range(self) -> int:
        """
        Returns the range of vision for the operator.

        Returns:
            int: The range of vision for the operator.
        """
        return self.sight_range
    
    def get_sight_distance(self) -> int:
        """
        Returns the maximum distance the operator can see.

        Returns:
            int: The maximum distance the operator can see.
        """
        return self.sight_distance
    
    def get_sight_angle(self) -> int:
        """
        Returns the angle of vision for the operator.

        Returns:
            int: The angle of vision for the operator.
        """
        return self.sight_angle
    
    def get_sight_direction(self) -> int:
        """
        Returns the direction the operator is facing.

        Returns:
            int: The direction the operator is facing.
        """
        return self.sight_direction
    
    def get_steps(self) -> int:
        """
        Returns the number of steps the operator can take in a turn.

        Returns:
            int: The number of steps the operator can take in a turn.
        """
        return self.steps
    
    def set_hp(self, hp: int):
        """
        Sets the health points of the operator.

        Args:
            hp (int): The health points of the operator.
        """
        self.hp = hp

    def set_mode(self, mode: str):
        """
        Sets the current mode of the operator.

        Args:
            mode (str): The current mode of the operator.
        """
        self.mode = mode

    def set_sight_range(self, sight_range: int):
        """
        Sets the range of vision for the operator.

        Args:
            sight_range (int): The range of vision for the operator.
        """
        self.sight_range = sight_range

    def set_sight_distance(self, sight_distance: int):
        """
        Sets the maximum distance the operator can see.

        Args:
            sight_distance (int): The maximum distance the operator can see.
        """
        self.sight_distance = sight_distance
    
    def set_sight_angle(self, sight_angle: int):
        """
        Sets the angle of vision for the operator.

        Args:
            sight_angle (int): The angle of vision for the operator.
        """
        self.sight_angle = sight_angle
    
    def set_sight_direction(self, sight_direction: int):
        """
        Sets the direction the operator is facing.

        Args:
            sight_direction (int): The direction the operator is facing.
        """
        self.sight_direction = sight_direction
    
    def set_steps(self, steps: int):
        """
        Sets the number of steps the operator can take in a turn.

        Args:
            steps (int): The number of steps the operator can take in a turn.
        """
        self.steps = steps
    
    def switch_mode(self, mode: str):
        """
        Switches the operator's mode.

        Args:
            mode (str): The mode to switch to.
        """
        self.mode = mode