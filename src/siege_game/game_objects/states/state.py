from abc import ABC
from enum import Enum
import uuid 

class StateData():
    class Side(Enum):
        DEFEND = uuid.uuid4()
        ATTACK = uuid.uuid4()

class State(ABC):
    pass

class SettingUpState(State):
    def __repr__(self):
        return f"SettingUpState"

class BattleState(State):
    
    def __init__(self):
        self.__side = StateData.Side.ATTACK

    def __repr__(self):
        return f"BattleState"
    
class StartState(State):
    def __repr__(self):
        return f"StartState"
    
class EndState(State):
    def __repr__(self):
        return f"EndState"
    
