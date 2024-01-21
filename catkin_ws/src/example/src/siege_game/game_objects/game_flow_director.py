from siege_game.game_objects.logger import Logger
from siege_game.game_objects.states.state import State, StartState, BattleState, SettingUpState, EndState
from collections import deque

class GameFlowDirector:
    logger = Logger('GameFlowDirector')

    def __init__(self):
        GameFlowDirector.logger.warning("<game_flow_director.py> Use get_instance class method to obtain the instance")
        self.__state = StartState()
        GameFlowDirector.logger.info(f"Current State: {self.__state}")
    
    def next_state(self):
        if isinstance(self.__state, StartState):
            self.__state = SettingUpState()
            GameFlowDirector.logger.info(f"Switch from StartState to SettingUpState")
        elif isinstance(self.__state, SettingUpState):
            self.__state = BattleState()
            GameFlowDirector.logger.info(f"Switch from SettingUpState to BattleState")
        elif isinstance(self.__state, BattleState):
            self.__state = EndState()
            GameFlowDirector.logger.info(f"Switch from BattleState to EndState")
        elif isinstance(self.__state, EndState):
            self.__state = StartState()
            GameFlowDirector.logger.info(f"Switch from EndState to StartState")

    def get_state(self):
        return self.__state

    # control operator sequence
    def create_battle_sequence_list(self, attacker_operators:deque, defender_operators:deque) -> deque:
        # Combine the two lists in an alternating sequence
        # list of Operator
        combined_sequence = [operator for pair in zip((attacker_operators), (defender_operators)) for operator in pair]
        return list(combined_sequence)
    