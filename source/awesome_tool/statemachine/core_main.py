from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.library_state import LibraryState
from statemachine.states.execution_state import ExecutionState

from state_machine_manager import StateMachineManager
from statemachine.storage.storage import Storage
import statemachine.singleton
from statemachine.enums import DataPortType, StateType

"""
A module to test different features of the state machine
"""

def state_without_path_test():
    state1 = ExecutionState("MyFirstState")
    state1.add_outcome("Success", 0)
    statemachine.singleton.state_machine_manager.root_state = state1
    statemachine.singleton.state_machine_execution_engine.start()
    pass


if __name__ == '__main__':

    state_without_path_test()

    #TODO: longterm
    # execution history
    # step back
    # validity checker