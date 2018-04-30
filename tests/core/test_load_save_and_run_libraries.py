import os
from os.path import join

# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.state_elements.data_port import InputDataPort, OutputDataPort
from rafcon.core.storage import storage
from rafcon.core.state_machine import StateMachine

# test environment elements
import testing_utils
import pytest


TEST_LIBRARY_PATH = os.path.join(testing_utils.RAFCON_TEMP_PATH_TEST_BASE, "test_libraries")


def setup_module(module=None):
    # set the test_libraries path temporarily to the correct value
    testing_utils.rewind_and_set_libraries({"temporary_libraries": TEST_LIBRARY_PATH})


def test_save_libraries(caplog):
    s = storage

    state1 = ExecutionState("library_execution_state1", path=testing_utils.TEST_SCRIPT_PATH,
                            filename="library_execution_state1.py")
    input_state1 = state1.add_input_data_port("data_input_port1", "float")
    output_state1 = state1.add_output_data_port("data_output_port1", "float")

    state2 = ExecutionState("library_execution_state2", path=testing_utils.TEST_SCRIPT_PATH,
                            filename="library_execution_state2.py")
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
    s.save_state_machine_to_path(StateMachine(state3), join(TEST_LIBRARY_PATH, "hierarchy_library"))

    # save execution state as state machine
    s.save_state_machine_to_path(StateMachine(state1), join(TEST_LIBRARY_PATH, "execution_library"))

    # save hierarchy state as nested state machines
    state3.name = "library_nested1"
    s.save_state_machine_to_path(StateMachine(state3), join(TEST_LIBRARY_PATH, "library_container", "library_nested1"),
                                 delete_old_state_machine=True)
    state3.name = "library_nested2"
    s.save_state_machine_to_path(StateMachine(state3),  join(TEST_LIBRARY_PATH, "library_container", "library_nested2"),
                                 delete_old_state_machine=True)
    testing_utils.assert_logger_warnings_and_errors(caplog)


def create_execution_state_library_state_machine():
    rafcon.core.singleton.library_manager.initialize()
    library_container_state = HierarchyState("libContainerState", state_id="libContainerState")
    lib_state = LibraryState("temporary_libraries", "execution_library", "0.1",
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
                                                                                           InputDataPort))
    library_container_state.add_data_flow(lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_output_port1",
                                                                                           OutputDataPort),
                                          library_container_state.state_id,
                                          lib_container_output)
    return StateMachine(library_container_state)


def test_execution_state_library(caplog):
    testing_utils.test_multithreading_lock.acquire()
    library_container_state_sm = create_execution_state_library_state_machine()

    rafcon.core.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    try:
        assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
        rafcon.core.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


def create_hierarchy_state_library_state_machine():
    rafcon.core.singleton.library_manager.initialize()
    library_container_state = HierarchyState("libContainerState", state_id="libContainerState")
    lib_state = LibraryState("temporary_libraries", "hierarchy_library", "0.1",
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
                                                                                           InputDataPort))
    library_container_state.add_data_flow(lib_state.state_id,
                                          lib_state.get_io_data_port_id_from_name_and_type("data_output_port1",
                                                                                           OutputDataPort),
                                          library_container_state.state_id,
                                          lib_container_output)
    return StateMachine(library_container_state)


def test_hierarchy_state_library(caplog):
    testing_utils.test_multithreading_lock.acquire()
    library_container_state_sm = create_hierarchy_state_library_state_machine()

    rafcon.core.singleton.state_machine_manager.add_state_machine(library_container_state_sm)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = library_container_state_sm.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    try:
        assert library_container_state_sm.root_state.output_data["data_output_port1"] == 42.0
        rafcon.core.singleton.state_machine_manager.remove_state_machine(library_container_state_sm.state_machine_id)
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


def test_save_nested_library_state(caplog):
    library_with_nested_library_sm = create_hierarchy_state_library_state_machine()

    storage.save_state_machine_to_path(library_with_nested_library_sm,
                                       join(TEST_LIBRARY_PATH, "library_with_nested_library"),
                                       delete_old_state_machine=True)
    testing_utils.assert_logger_warnings_and_errors(caplog)


def test_nested_library_state_machine(caplog):
    testing_utils.test_multithreading_lock.acquire()
    # TODO: the library_manager is initialized a second time here
    rafcon.core.singleton.library_manager.initialize()
    nested_library_state = LibraryState("temporary_libraries", "library_with_nested_library", "0.1",
                                        "nested_library_state_name", "nested_library_state_id")
    state_machine = StateMachine(nested_library_state)

    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = state_machine.state_machine_id
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()

    # print output_data["data_output_port1"]
    try:
        assert nested_library_state.output_data["data_output_port1"] == 42.0
        rafcon.core.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)
        testing_utils.assert_logger_warnings_and_errors(caplog)
    finally:
        testing_utils.test_multithreading_lock.release()


def test_rafcon_library_path_variable(caplog):
    rafcon.core.config.global_config.set_config_value("LIBRARY_PATHS", {})
    os.environ['RAFCON_LIBRARY_PATH'] = os.path.join(testing_utils.LIBRARY_SM_PATH, 'generic')
    rafcon.core.singleton.library_manager.initialize()
    os.environ['RAFCON_LIBRARY_PATH'] = ""
    libraries = rafcon.core.singleton.library_manager.libraries
    assert 'generic' in libraries
    assert isinstance(libraries['generic'], dict)


def teardown_module(module=None):
    testing_utils.reload_config(gui_config=False)


if __name__ == '__main__':
    setup_module()
    # test_save_libraries(None)
    # test_execution_state_library(None)
    # test_hierarchy_state_library(None)
    # test_save_nested_library_state(None)
    # test_nested_library_state_machine(None)
    pytest.main(['-s', __file__])
