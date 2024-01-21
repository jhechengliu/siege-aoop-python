import json
from siege_game.game_objects.map.map_objects import door, floor, window, soft_wall, wall, entrance, barrier
from siege_game.game_objects.map import map
from siege_game.game_objects.logger import Logger
from typing import Dict, List

class MapBuilder:
    logger = Logger('MapBuilder')
    """
    NOTE: Use get_instance method to get the instance, DONT use constructor 
    A Builder that builds a Map object with its esseitials stuff such as the map,
    the players on it, the data flow and game flow of it.
    """
    
    def __init__(self, file_name:str, game_id:str):
        """
        Make a builder
        """ 
        self.__game_id = game_id
        self.__map_json = None
        self.__map_width = 0
        self.__map_height = 0
        self.__load_map(file_name)
        self.__map_data = {}
        self.__id_to_map_object(self.__map_json["map"], self.__map_json['mapping'])

    def get_map(self) -> map.Map:
        """
        Get the map the builder builds

        Returns:
            (Map) The map object which the MapBuilder builds
        """
        # map init: self, map_data:dict, map_width:int, map_height:int, defender_count:int, attacker_count:int
        return map.Map(self.__map_data, self.__map_width, self.__map_height, 2, 2, self.__game_id)

    def __load_map(self, file_name) -> None:
        """
        Loads the json file and put the data into the __map_json attribute

        Attributes:
            file_name (string): the json file's name (without '.json') in the Assets/Data Folder

        Returns:
            (None)
        """
        with open('/home/rsa/siege-aoop-python/Assets/Data/' + file_name + '.json', 'r') as json_file:
            # Load the JSON content into a Python dictionary
            self.__map_json = json.load(json_file)
            MapBuilder.logger.info("Raw map file loaded")
            self.__map_width = len(self.__map_json['map'][0])
            self.__map_height = len(self.__map_json['map'])
            MapBuilder.logger.info(f"Map size loaded {self.__map_width} * {self.__map_height}")
            

    def __id_to_map_object(self, array: List, map_data: List) -> None:
        """
        Change the list from number to map objects by looking at the map dictionary

        Attributes:
            array (2-dim list): the outside dim list is the height while the inner side is the width
            (array[y][x])
            array[0][0] is at the top left of the map, array[0][1] is right below array[0][0] and array[1][0]
            is at the right of array[0][0]

            map (dictionary): keys are numbers or id written in the json file while value are the object type represent 
            using string. Object types consist of 5 types:
            'floor', 'wall', 'soft_wall', 'door', 'window'

        Returns:
            (None) but the value of the self.__map_array will change after doing this

        Throws:
            NoSuchJsonObjectTypeError: thrown when array has non recognized id in array or map has unknown object type strings 
        """

        x_len = len(array[0])
        y_len = len(array)
        for y in range(y_len):
            for x in range(x_len):
                location = (x, y)
                
                try:
                    object_type = map_data[array[y][x]]
                except KeyError:
                    raise NoSuchJsonObjectTypeError()
                
                if (object_type == "wall"): 
                    self.__map_data[location] = wall.Wall(location)
                elif (object_type == "floor"):
                    self.__map_data[location] = floor.Floor(location)
                elif (object_type == "softWall"):
                    self.__map_data[location] = soft_wall.SoftWall(location)
                elif (object_type == "window"):
                    self.__map_data[location] = window.Window(location)
                elif (object_type == "door"):
                    self.__map_data[location] = door.Door(location)
                elif (object_type == "entrance"):
                    self.__map_data[location] = entrance.Entrance(location)
                elif (object_type == "barrier"):
                    self.__map_data[location] = barrier.Barrier(location)
                else:
                    raise NoSuchJsonObjectTypeError()

        MapBuilder.logger.info("<map_build.py> Map ID -> Obj Done")


class NoSuchJsonObjectTypeError(Exception):
    """
    Called when someone does something bad in replacing id with objects in MapBuilder class __id_to_map_object methods
    """
    pass

    
    

