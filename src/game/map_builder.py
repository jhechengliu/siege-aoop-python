import json
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
            A new MapBuilder Class
        """
        self.__map_json = None
        self.__load_map(file_name)

    def __load_map(self, file_name) -> None:
        """
        Loads the json file and put the data into the __map_json attribute

        Attributes:
            file_name (string): the json file's name (without '.json') in the Assets/Data Folder

        Returns:
            None
        """
        with open('././Assets/Data/' + file_name + '.json', 'r') as json_file:
            # Load the JSON content into a Python dictionary
            self.__map_json = json.load(json_file)
            print(self.__map_json)

    def __id_to_map_object(array: list, map: dict):
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
            None, but the value of the array will change after doing this

        Throws:
            NoSuchJsonObjectTypeError: thrown when array has non recognized id in array or map has unknown object type strings 
        """

        width = len(array[0])
        length = len(array)
        for length_index in range(width):
            for width_index in range(length):
                try:
                    object_type = map(str(array[length][width]))
                except KeyError:
                    raise NoSuchJsonObjectTypeError()
                
                if (object_type == "wall"):
                    pass
                elif (object_type == "floor"):
                    pass
                elif (object_type == "soft_wall"):
                    pass
                elif (object_type == "window"):
                    pass
                elif (object_type == "door"):
                    pass
                else:
                    raise NoSuchJsonObjectTypeError()
                

class NoSuchJsonObjectTypeError(Exception):
    """
    Used in replacing id with objects in MapBuilder class __id_to_map_object methods
    """
    pass

    
    

