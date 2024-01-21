from typing import List
import math
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.map.map_objects import floor, wall, door, window, soft_wall, entrance, barrier, barrier

class SightChecker:
    def __init__(self) -> None:
        self.__map_data = None

    def check_sight(self, map_data:dict, location_a:List[float], location_b:List[float]) -> bool:
        self.__map_data = map_data
        delta_x = (location_b[0] - location_a[0]) / 100
        delta_y = (location_b[1] - location_a[1]) / 100
        point = location_a

        for i in range(0, 100):
            point = point[0] + delta_x, point[1] + delta_y
            if ((self.__map_data[ ( math.floor(point[0]), math.floor(point[1]) ) ]).get_is_transparent() == False):
                return False
        
        return True