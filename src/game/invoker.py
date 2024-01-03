import warnings

class Invoker():
    """
        
    """
    __instance = None
    def __init__(self) -> None:
        warnings.warn("Use get_instance class method to obtain the instance", UserWarning)

    @classmethod
    def get_instance():
        if Invoker.__instance == None:
            Invoker.__instance = Invoker()
        
        return Invoker.__instance