import pytest
from pytest import raises
from os.path import dirname, join, realpath

from awesome_tool.statemachine.states.execution_state import ExecutionState
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.library_state import LibraryState
import awesome_tool.statemachine.singleton
from awesome_tool.statemachine.states.state import DataPortType
from awesome_tool.statemachine.storage.storage import StateMachineStorage
from awesome_tool.statemachine.state_machine import StateMachine
import variables_for_pytest
import awesome_tool.statemachine.config

def setup_module(module=None):
    # set the test_libraries path temporarily to the correct value
    library_paths = awesome_tool.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["test_libraries"] = join(dirname(dirname(realpath(__file__))), "test_scripts", "test_libraries")


def test_save_libraries():
    s = StateMachineStorage("../test_scripts/test_libraries")

    state1 = ExecutionState("library_execution_state1", path="../test_scripts", filename="library_execution_state1.py")
    input_state1 = state1.add_input_data_port("data_input_port1", "float")
    output_state1 = state1.add_output_data_port("data_output_port1", "float")

    state2 = ExecutionState("library_execution_state2", path="../test_scripts", filename="library_execution_state2.py")
    input_state2 = state2.add_input_data_port("data_input_port1", "float")
    output_state2 = state2.add_output_data_port("data_output_port1", "float")

    state3 = HierarchyState("library_hierarchy_state1", path="../test_scripts", filename="library_hierarchy_state.py")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)

    state3.add_transition(state1.state_id, 0, state2.state_id, None)
    state3.add_transition(state2.state_id, 0, state3.state_id, 0)
    input_state3 = state3.add_input_data_port("data_input_port1", "float")
    output_state3 = state3.add_output_data_port("data_output_port1", "float")
    state3.add_data_flow(state3.state_id,
                         input_state3,
                         state1.state_id,
                         input_state1)
    state3.add_data_flow(state1.state_id,
                         output_state1,
                         state2.state_id,
                         input_state2)
    state3.add_data_flow(state2.state_id,
                         output_state2,
                         state3.state_id,
                         output_state3)

    # save hierarchy state as state machine
    s.save_statemachine_as_yaml(StateMachine(state3), "../test_scripts/test_libraries/hierarchy_library", "0.1")

    # save execution state as state machine
    s.save_statemachine_as_yaml(StateMachine(state1), "../test_scripts/test_libraries/execution_library", "0.1")

    # save hierarchy state as nested state machines
    state3.name = "library_nested1"
    s.save_statemachine_as_yaml(StateMachine(state3), "../test_scripts/test_libraries/library_container/library_nested1", "0.1")
    state3.name = "library_nested2"
    s.save_statemachine_as_yaml(StateMachine(state3), "../test_scripts/test_libraries/library_container/library_nested2", "0.1")


def create_hierarchy_state_library_state_machine():
    awesome_tool.statemachine.singleton.library_manager.initialize()
    library_container_state = HierarchyState("LibContainerState", path="../test_scripts",
                                             filename="hierarchy_state.py")
    lib_state = LibraryState("test_libraries", "hierarchy_library", "0.1", "library_state")
    library_container_state.add_state(lib_state)
    library_container_state.set_start_state(lib_state.state_id)

    library_container_state.add_transition(lib_state.state_id, 0, library_container_state.state_id, 0)
    lib_container_input = library_container_state.add_input_data_port("data_input_port1", "float", 22.0)
    lib_container_output = library_container_state.add_output_data_port("data_output_port1", "float")
    library_container_state.add_data_flow(library_container_state.state_id,
                                          lib_container_input,
                                          lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_input_port1",
                                                                                           DataPortType.INPUT))
    library_container_state.add_data_flow(lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_output_port1",
                                                                                           DataPortType.OUTPUT),
                                          library_container_state.state_id,
                                          lib_container_output)
    return StateMachine(library_container_state)


def create_execution_state_library_state_machine():
    awesome_tool.statemachine.singleton.library_manager.initialize()
    library_container_state = HierarchyState("LibContainerState", path="../test_scripts",
                                             filename="hierarchy_state.py")
    lib_state = LibraryState("test_libraries", "execution_library", "0.1", "library_state")
    library_container_state.add_state(lib_state)
    library_container_state.set_start_state(lib_state.state_id)

    library_container_state.add_transition(lib_state.state_id, 0, library_container_state.state_id, 0)
    lib_container_input = library_container_state.add_input_data_port("data_input_port1", "float", 32.0)
    lib_container_output = library_container_state.add_output_data_port("data_output_port1", "float")
    library_container_state.add_data_flow(library_container_state.state_id,
                                          lib_container_input,
                                          lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_input_port1",
                                                                                           DataPortType.INPUT))
    library_container_state.add_data_flow(lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_output_port1",
                                                                                           DataPortType.OUTPUT),
                                          library_container_state.state_id,
                                          lib_container_output)
    return StateMachine(library_container_state)


def test_save_nested_library_state():
    library_with_nested_library_sm = create_hierarchy_state_library_state_machine()

    awesome_tool.statemachine.singleton.global_storage.save_statemachine_as_yaml(
        library_with_nested_library_sm, "../test_scripts/test_libraries/library_with_nested_library", "0.1")


def test_hierarchy_state_library():
    variables_for_pytest.test_multithrading_lock.acquire()
    library_container_state_sm = create_hierarchy_state_library_state_machine()

    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    library_container_state_sm.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    # print output_data["data_output_port1"]
    assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()


def test_execution_state_library():
    variables_for_pytest.test_multithrading_lock.acquire()
    library_container_state_sm = create_execution_state_library_state_machine()

    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    library_container_state_sm.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    # print output_data["data_output_port1"]
    assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()


def test_nested_library_state_machine():
    variables_for_pytest.test_multithrading_lock.acquire()
    awesome_tool.statemachine.singleton.library_manager.initialize()
    nested_library_state = LibraryState("test_libraries", "library_with_nested_library", "0.1", "library_state_name")
    state_machine = StateMachine(nested_library_state)

    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    nested_library_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    # print output_data["data_output_port1"]
    assert nested_library_state.output_data["data_output_port1"] == 42.0
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    variables_for_pytest.test_multithrading_lock.release()


def teardown_module(module=None):
    pass


if __name__ == '__main__':
    setup_module()
    test_save_libraries()
    # print "\n################### next function #########################\n"
    test_save_nested_library_state()
    # print "\n################### next function #########################\n"
    test_hierarchy_state_library()
    # print "\n################### next function #########################\n"
    test_execution_state_library()
    # print "\n################### next function #########################\n"
    test_nested_library_state_machine()
    teardown_module()
