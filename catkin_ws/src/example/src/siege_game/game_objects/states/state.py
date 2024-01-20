from abc import ABC


class State(ABC):
    pass

class SettingUpState(State):
    def __init__(self):
        pass
    
    def __repr__(self):
        return f"SettingUpState"

class BattleState(State):
    
    def __init__(self):
        pass

    def __repr__(self):
        return f"BattleState"
    
class StartState(State):
    def __init__(self):
        pass
    def __repr__(self):
        return f"StartState"
    
class EndState(State):
    def __init__(self):
        pass
    def __repr__(self):
        return f"EndState"
    
