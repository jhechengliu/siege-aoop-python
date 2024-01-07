import json
from game.map.map_objects import door, floor, window, soft_wall, wall
import pprint

class MapBuilder:
    """
    NOTE: Use get_instance method to get the instance, DONT use constructor 
    A Builder that builds a Map object with its esseitials stuff such as the map,
    the players on it, the data flow and game flow of it.

    Attributes:
        __map_json (dictionary): stores the json file map design and the label on it

    Methods:
        get_instance: use this to get the instance of this class because there can only be 
        one MapBuilder at the same time

        __load_map: well, load a map and put the data into the __map_json attribute
        __id_to_map_object: chnage the array content from numbers to map objects such as Wall object Floor object etc
    """
    instance = None

    @classmethod
    def get_instance(file_name):
        """
        Only use this to get the instance of this class

        Attributes:
            file_name (string): the json file's name (without '.json') in the Assets/Data Folder

        Returns:
            if there is no instance made before, make a new instance and returns it
            if not, returns the already made instance
        """
        if (MapBuilder.instance == None):
            MapBuilder.instance = MapBuilder(file_name)
        
        return MapBuilder.instance
    
    def __init__(self, file_name):
        """
        Make a builder

        Attributes:
            file_name (string): the json file's name (without '.json') in the Assets/Data Folder

        Returns:
            (MapBuilder) A new MapBuilder Class
        """ 
        self.__map_json = None
        self.__load_map(file_name)
        self.__map = {}
        self.__id_to_map_object(self.__map_json["map"], self.__map_json)

    def __load_map(self, file_name) -> None:
        """
        Loads the json file and put the data into the __map_json attribute

        Attributes:
            file_name (string): the json file's name (without '.json') in the Assets/Data Folder

        Returns:
            (None)
        """
        with open('././Assets/Data/' + file_name + '.json', 'r') as json_file:
            # Load the JSON content into a Python dictionary
            self.__map_json = json.load(json_file)
            print(self.__map_json)

    def __id_to_map_object(self, array: list, map: dict) -> None:
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
        print("x_len", x_len, "y_len", y_len)
        for y in range(y_len):
            for x in range(x_len):
                location = (x, y)
                
                try:
                    object_type = map[str(array[y][x])]
                    print(object_type)
                except KeyError:
                    raise NoSuchJsonObjectTypeError()
                
                print(x, y, object_type)
                if (object_type == "wall"): 
                    self.__map[location] = wall.Wall(location)
                elif (object_type == "floor"):
                    self.__map[location] = floor.Floor(location)
                elif (object_type == "softWall"):
                    self.__map[location] = soft_wall.SoftWall(location)
                elif (object_type == "window"):
                    self.__map[location] = window.Window(location)
                elif (object_type == "door"):
                    self.__map[location] = door.Door(location)
                else:
                    raise NoSuchJsonObjectTypeError()
                
        pprint.pprint(self.__map, indent=4, sort_dicts=True)


class NoSuchJsonObjectTypeError(Exception):
    """
    Called when someone does something bad in replacing id with objects in MapBuilder class __id_to_map_object methods
    """
    pass

    
    

