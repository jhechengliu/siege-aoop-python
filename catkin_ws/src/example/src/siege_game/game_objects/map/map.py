
from siege_game.game_objects.map_data_processor import MapDataProcessor
from siege_game.game_objects.game_data_publisher import GameDataPublisher
from siege_game.game_objects.game_flow_director import GameFlowDirector
from collections import deque
from siege_game.game_objects.pawn.attacker import Attacker
from siege_game.game_objects.pawn.defender import Defender
from siege_game.game_objects.pawn.shooting_system import ShootingSystem
from siege_game.game_objects.pawn.walking_system import WalkingSystem
from siege_game.game_objects.pawn.sight_checker import SightChecker
from typing import List
from siege_game.game_objects.logger import Logger

class Map:
    logger = Logger('map')

    def __init__(self, map_data:dict, map_width:int, map_height:int, defender_count:int, attacker_count:int):
        """
        Don't use constructor to init this class, use get_instance method instead
        """
        Map.logger.warning("<map.py> Use get_instance class method to obtain the instance")
        self.__map_data = map_data
        self.__shooting_system = ShootingSystem()
        self.__walking_system = WalkingSystem()
        self.__sight_checker = SightChecker()
        self.__defenders = deque()
        self.__attackers = deque()
        self.__map_width = map_width
        self.__map_height = map_height
        self.__map_data_processor = MapDataProcessor()
        self.__game_data_publisher = GameDataPublisher()
        self.__game_flow_director = GameFlowDirector()
        self.__max_defender_count = defender_count
        self.__max_attacker_count = attacker_count
    
    def print_map(self):
        Map.logger.info(f"Printing the current map... (Size: {self.__map_width} * {self.__map_height})")
        for y in range(self.__map_height):
            for x in range(self.__map_width):
                print('{0:<20}'.format(f'{self.__map[(x, y)]}'), end=" ")
            print()
   
    def add_attacker(self, location:List[float]) -> None:
        self.__attackers.append(Attacker(location, self.__shooting_system, self.__walking_system))

    def add_defender(self, location:List[float]) -> None:
        self.__defenders.append(Defender(location, self.__shooting_system, self.__walking_system))

    def map_status(self):
        Map.logger.info("--- Map Status ---")
        Map.logger.info(f"Attackers: {self.__attackers}")
        Map.logger.info(f"Defenders: {self.__defenders}")
        Map.logger.info("--- End of Map Status ---")

    def get_map_object(self, location:List[int]):
        return self.__map_data[(location[0], location[1])]
    
    def call_shooting_system(self, attacker_operator_index:int, defender_operator_index:int) -> None:
        # use the index to get the operator
        attacker_operator = self.__attackers[attacker_operator_index]
        defender_operator = self.__defenders[defender_operator_index]
        self.__

    def get_map_data(self) -> dict:
        return self.__map_data
    
    def get_shooting_system(self) -> ShootingSystem:
        return self.__shooting_system
    
    def get_walking_system(self) -> WalkingSystem:
        return self.__walking_system
    
    def get_sight_checker(self) -> SightChecker:
        return self.__sight_checker
    
    def get_map_data_processor(self) -> MapDataProcessor:
        return self.__map_data_processor
    
    def get_game_data_publisher(self) -> GameDataPublisher:
        return self.__game_data_publisher
    
    def get_game_flow_director(self) -> GameFlowDirector:
        return self.__game_flow_director
    
    def get_max_defender_count(self) -> int:
        return self.__max_defender_count
    
    def get_max_attacker_count(self) -> int:
        return self.__max_attacker_count
    
    def get_defenders(self) -> deque:
        return self.__defenders
    
    def get_attackers(self) -> deque:
        return self.__attackers
    
    def get_deffender(self, index:int) -> Defender:
        return self.__defenders[index]
    
    def get_attacker(self, index:int) -> Attacker:
        return self.__attackers[index]
    
    def get_map_width(self) -> int:
        return self.__map_width
    
    def get_map_height(self) -> int:
        return self.__map_height
    
    def get_map_object(self, location:List[int]):
        return self.__map[(location[0], location[1])]