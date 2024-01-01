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

    def __load_map(self, file_name):
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
    
    

