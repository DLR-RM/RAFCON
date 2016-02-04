from os.path import dirname, join, realpath

# core elements
from rafcon.statemachine.states.execution_state import ExecutionState
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.library_state import LibraryState
from rafcon.statemachine.states.state import DataPortType
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.state_machine import StateMachine

# singleton elements
import rafcon.statemachine.singleton
import rafcon.mvc.singleton

# test environment elements
import testing_utils
import pytest


def setup_module(module=None):
    # set the test_libraries path temporarily to the correct value
    testing_utils.remove_all_libraries()
    library_paths = rafcon.statemachine.config.global_config.get_config_value("LIBRARY_PATHS")
    library_paths["test_libraries"] = testing_utils.get_test_sm_path("test_libraries")


def test_save_libraries(caplog):
    s = StateMachineStorage(testing_utils.get_test_sm_path("test_libraries"))

    state1 = ExecutionState("library_execution_state1", path=testing_utils.TEST_SM_PATH, filename="library_execution_state1.py")
    input_state1 = state1.add_input_data_port("data_input_port1", "float")
    output_state1 = state1.add_output_data_port("data_output_port1", "float")

    state2 = ExecutionState("library_execution_state2", path=testing_utils.TEST_SM_PATH, filename="library_execution_state2.py")
    input_state2 = state2.add_input_data_port("data_input_port1", "float")
    output_state2 = state2.add_output_data_port("data_output_port1", "float")

    state3 = HierarchyState("library_hierarchy_state1")
    state3.add_state(state1)
    state3.add_state(state2)
    state3.set_start_state(state1.state_id)

    state3.add_transition(state1.state_id, 0, state2.state_id, None)
    state3.add_transition(state2.state_id, 0, state3.state_id, 0)
    input_state3 = state3.add_input_data_port("data_input_port1", "float", 1.0)
    output_state3 = state3.add_output_data_port("data_output_port1", "float", 2.0)
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
    s.save_statemachine_to_path(StateMachine(state3), testing_utils.get_test_sm_path("test_libraries/hierarchy_library"),
                                "0.1")

    # save execution state as state machine
    s.save_statemachine_to_path(StateMachine(state1), testing_utils.get_test_sm_path("test_libraries/execution_library"),
                                "0.1")

    # save hierarchy state as nested state machines
    state3.name = "library_nested1"
    s.save_statemachine_to_path(StateMachine(state3),
                                testing_utils.get_test_sm_path("test_libraries/library_container/library_nested1"), "0.1")
    state3.name = "library_nested2"
    s.save_statemachine_to_path(StateMachine(state3),
                                testing_utils.get_test_sm_path("test_libraries/library_container/library_nested2"), "0.1")
    # test_utils.assert_logger_warnings_and_errors(caplog)


def create_execution_state_library_state_machine():
    rafcon.statemachine.singleton.library_manager.initialize()
    library_container_state = HierarchyState("libContainerState", state_id="libContainerState")
    lib_state = LibraryState("test_libraries", "execution_library", "0.1",
                             "library_execution_state", state_id="library_execution_state")
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


def test_execution_state_library(caplog):
    testing_utils.test_multithrading_lock.acquire()
    library_container_state_sm = create_execution_state_library_state_machine()

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
    testing_utils.test_multithrading_lock.release()
    # test_utils.assert_logger_warnings_and_errors(caplog)


def create_hierarchy_state_library_state_machine():
    rafcon.statemachine.singleton.library_manager.initialize()
    library_container_state = HierarchyState("libContainerState", state_id="libContainerState")
    lib_state = LibraryState("test_libraries", "hierarchy_library", "0.1",
                             "library_hierarchy_state", state_id="library_hierarchy_state")
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


def test_hierarchy_state_library(caplog):
    testing_utils.test_multithrading_lock.acquire()
    library_container_state_sm = create_hierarchy_state_library_state_machine()

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
    testing_utils.test_multithrading_lock.release()
    # test_utils.assert_logger_warnings_and_errors(caplog)


def test_save_nested_library_state(caplog):
    library_with_nested_library_sm = create_hierarchy_state_library_state_machine()

    rafcon.statemachine.singleton.global_storage.save_statemachine_to_path(
        library_with_nested_library_sm, testing_utils.get_test_sm_path("test_libraries/library_with_nested_library"), "0.1")
    # test_utils.assert_logger_warnings_and_errors(caplog)


def test_nested_library_state_machine(caplog):
    testing_utils.test_multithrading_lock.acquire()
    # TODO: the library_manager is initialized a second time here
    rafcon.statemachine.singleton.library_manager.initialize()
    nested_library_state = LibraryState("test_libraries", "library_with_nested_library", "0.1",
                                        "nested_library_state_name", "nested_library_state_id")
    state_machine = StateMachine(nested_library_state)

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.statemachine.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    rafcon.statemachine.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    assert nested_library_state.output_data["data_output_port1"] == 42.0
    rafcon.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
    testing_utils.test_multithrading_lock.release()
    # test_utils.assert_logger_warnings_and_errors(caplog)


def teardown_module(module=None):
    testing_utils.reload_config()


if __name__ == '__main__':
    setup_module()
    test_save_libraries(None)
    test_execution_state_library(None)
    test_hierarchy_state_library(None)
    test_save_nested_library_state(None)
    test_nested_library_state_machine(None)
    # pytest.main([__file__])
