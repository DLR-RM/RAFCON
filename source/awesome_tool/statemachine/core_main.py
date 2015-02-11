from statemachine.states.hierarchy_state import HierarchyState
from statemachine.states.library_state import LibraryState
from statemachine.states.execution_state import ExecutionState

from state_machine_manager import StateMachineManager
from external_modules.external_module import ExternalModule
from statemachine.storage.storage import Storage
import statemachine.singleton
from statemachine.enums import DataPortType, StateType


def ros_external_module_test():
    state1 = ExecutionState("MyFirstState", path="../../test_scripts", filename="ros_test_state.py")
    state1.add_outcome("MyFirstOutcome", 3)
    state1.add_input_data_port("MyFirstDataInputPort", "str")
    state1.add_output_data_port("MyFirstDataOutputPort", "float")

    input_data = {"MyFirstDataInputPort": "input_string"}
    output_data = {"MyFirstDataOutputPort": None}
    state1.input_data = input_data
    state1.output_data = output_data

    em = ExternalModule(name="ros", module_name="ros_external_module", class_name="RosModule")
    statemachine.singleton.external_module_manager.add_external_module(em)
    statemachine.singleton.external_module_manager.external_modules["ros"].connect([])
    statemachine.singleton.external_module_manager.external_modules["ros"].start()

    sm = StateMachineManager(state1)
    sm.start()


def state_without_path_test():
    state1 = ExecutionState("MyFirstState")
    state1.add_outcome("Success", 0)
    statemachine.singleton.state_machine_manager.root_state = state1
    statemachine.singleton.state_machine_execution_engine.start()
    pass


if __name__ == '__main__':

    state_without_path_test()
    #ros_external_module_test()

    #TODO: longterm
    # execution history
    # step back
    # validity checker