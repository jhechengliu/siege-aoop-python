from typing import List
from siege_game.game_objects.logger import Logger
from siege_game.game_objects.map.map_objects import floor, wall, door, window, soft_wall, entrance, barrier, barrier

class SightChecker:
    def __init__(self, map:dict) -> None:
        self.__location_a = None
        self.__location_b = None
        self.__map = map

    def check_sight(self, location_a:List[float], location_b:List[float]) -> bool:
        self.__location_a = location_a
        self.__location_b = location_b
        start_x = self.__location_b[0] - self.__location_a[0]
        start_y = self.__location_b[1] - self.__location_a[1]
        delta_x = self.__location_b[0] - self.__location_a[0]
        delta_y = self.__location_b[1] - self.__location_a[1]
        point = start_x, start_y

        #for loop x times, check if all clear
        #if all clear, return true
        #else return false

        for i in range(0, 1000):
            point = point[0] + delta_x, point[1] + delta_y
            if (self.__map[point] == 'wall'):
                return False