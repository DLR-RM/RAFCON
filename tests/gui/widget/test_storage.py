import os
import logging
import shutil
import pytest
# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


def create_state_machine():
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.state_machine import StateMachine

    print "create models"

    logger.setLevel(logging.VERBOSE)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    state1 = ExecutionState('State1', state_id='STATE1')
    output_state1 = state1.add_output_data_port("output", "int")
    input_state1 = state1.add_input_data_port("input", "str", "zero")
    state2 = ExecutionState('State2', state_id='STATE2')
    input_par_state2 = state2.add_input_data_port("par", "int", 0)
    output_res_state2 = state2.add_output_data_port("res", "int")
    state4 = HierarchyState(name='Nested', state_id='NESTED')
    state4.add_outcome('GoGo')
    output_state4 = state4.add_output_data_port("out", "int")
    state5 = ExecutionState('Nested2', state_id='NESTED2')
    state5.add_outcome('HereWeGo')
    input_state5 = state5.add_input_data_port("in", "int", 0)
    state3 = HierarchyState(name='State3', state_id='STATE3')
    input_state3 = state3.add_input_data_port("input", "int", 0)
    output_state3 = state3.add_output_data_port("output", "int")
    state3.add_state(state4)
    state3.add_state(state5)
    state3.set_start_state(state4)
    state3.add_scoped_variable("share", "int", 3)
    state3.add_transition(state4.state_id, 0, state5.state_id, None)
    state3.add_transition(state5.state_id, 0, state3.state_id, 0)
    state3.add_data_flow(state4.state_id, output_state4, state5.state_id, input_state5)
    state3.add_outcome('Branch1')
    state3.add_outcome('Branch2')
    ctr_state = HierarchyState(name="Container", state_id='CONT2')
    ctr_state.add_state(state1)
    ctr_state.add_state(state2)
    ctr_state.add_state(state3)
    input_ctr_state = ctr_state.add_input_data_port("ctr_in", "str", "zero")
    output_ctr_state = ctr_state.add_output_data_port("ctr_out", "int")
    ctr_state.set_start_state(state1)
    ctr_state.add_transition(state1.state_id, 0, state2.state_id, None)
    ctr_state.add_transition(state2.state_id, 0, state3.state_id, None)
    ctr_state.add_transition(state3.state_id, 0, ctr_state.state_id, 0)
    ctr_state.add_data_flow(state1.state_id, output_state1, state2.state_id, input_par_state2)
    ctr_state.add_data_flow(state2.state_id, output_res_state2, state3.state_id, input_state3)
    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, state1.state_id, input_state1)
    ctr_state.add_data_flow(state3.state_id, output_state3, ctr_state.state_id, output_ctr_state)
    ctr_state.name = "Container"
    ctr_state.add_input_data_port("input", "str", "default_value1")
    ctr_state.add_input_data_port("pos_x", "str", "default_value2")
    ctr_state.add_input_data_port("pos_y", "str", "default_value3")

    ctr_state.add_output_data_port("output", "str", "default_value1")
    ctr_state.add_output_data_port("result", "str", "default_value2")

    scoped_variable1_ctr_state = ctr_state.add_scoped_variable("scoped", "str", "default_value1")
    scoped_variable2_ctr_state = ctr_state.add_scoped_variable("my_var", "str", "default_value1")
    scoped_variable3_ctr_state = ctr_state.add_scoped_variable("ctr", "int", 42)

    ctr_state.add_data_flow(ctr_state.state_id, input_ctr_state, ctr_state.state_id, scoped_variable1_ctr_state)
    ctr_state.add_data_flow(state1.state_id, output_state1, ctr_state.state_id, scoped_variable3_ctr_state)
    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4,
                  'Nested2': state5}
    sm = StateMachine(ctr_state)
    return sm


def create_models():
    import rafcon.core.singleton
    import rafcon.gui.singleton

    sm = create_state_machine()
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    # give gui time to create the state machine
    testing_utils.wait_for_gui()

    for sm_in in rafcon.core.singleton.state_machine_manager.state_machines.values():
        rafcon.core.singleton.state_machine_manager.remove_state_machine(sm_in.state_machine_id)
    # give the gui time to remove the state machine
    testing_utils.wait_for_gui()
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    # wait until model is created, otherwise gui will crash
    testing_utils.wait_for_gui()
    rafcon.core.singleton.state_machine_manager.active_state_machine_id = sm.state_machine_id


def on_save_activate(state_machine_m, logger):
    from rafcon.core.storage import storage

    if state_machine_m is None:
        return
    save_path = state_machine_m.state_machine.base_path
    if save_path is None:
        return

    logger.debug("Saving state machine to {0}".format(save_path))
    storage.save_state_machine_to_path(state_machine_m.state_machine,
                                      state_machine_m.state_machine.base_path, delete_old_state_machine=False)

    state_machine_m.root_state.store_meta_data()
    logger.debug("Successfully saved graphics meta data.")


def save_state_machine(with_gui=True):
    import rafcon
    from rafcon.core.singleton import state_machine_execution_engine
    import rafcon.gui.singleton as gui_singleton
    from rafcon.core.storage import storage

    path = testing_utils.get_unique_temp_path()
    if with_gui:
        call_gui_callback(create_models)
    else:
        create_models()

    state_machine = rafcon.core.singleton.state_machine_manager.get_active_state_machine()
    if with_gui:
        sm_model = rafcon.gui.singleton.state_machine_manager_model.state_machines[state_machine.state_machine_id]
        menubar_ctrl = gui_singleton.main_window_controller.get_controller('menu_bar_controller')
        # sm_model.state_machine.base_path = path
        call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, path)
        # call_gui_callback(menubar_ctrl.on_quit_activate, None)
        call_gui_callback(check_that_all_files_are_there, state_machine, with_print=False)
    else:
        storage.save_state_machine_to_path(state_machine, path, delete_old_state_machine=False)
        check_that_all_files_are_there(state_machine, with_print=False)


def check_file(file_path, kind, missing_elements=None, existing_elements=None):
    if os.path.isfile(file_path):
        logger.debug("%s: '%s' exists" % (kind, file_path))
        if existing_elements is not None:
            existing_elements.append(file_path)
        return True
    else:
        logger.debug("%s: '%s' DOESN'T exist" % (kind, file_path))
        if missing_elements is not None:
            missing_elements.append(file_path)
        return False


def check_folder(folder_path, kind, missing_elements=None, existing_elements=None):
    if os.path.exists(folder_path):
        logger.debug("%s: '%s' exists" % (kind, folder_path))
        if existing_elements is not None:
            existing_elements.append(folder_path)
        return True
    else:
        logger.debug("%s: '%s' DOESN'T exist" % (kind, folder_path))
        if missing_elements is not None:
            missing_elements.append(folder_path)
        return False


def check_state_machine_storage(state_machine, path, missing_elements, existing_elements=None, check_meta_data=False):
    from rafcon.core.storage import storage
    # check state machine folder exists
    check_folder(path, "state machine path", missing_elements, existing_elements)

    # check state-meta data exists (transitions and so on)
    file_path = os.path.join(path, storage.STATEMACHINE_FILE)
    check_file(file_path, "state machine data", missing_elements, existing_elements)

    # check if optional gui-meta-data exists
    if check_meta_data:
        # gtk gui meta data
        file_path = os.path.join(path, storage.FILE_NAME_META_DATA)
        check_file(file_path, "meta data", missing_elements, existing_elements)

    if state_machine.root_state:
        check_state_storage(state_machine.root_state, path, missing_elements, existing_elements, check_meta_data)


def check_state_storage(state, parent_path, missing_elements, existing_elements=None, check_meta_data=False):
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.storage import storage
    from rafcon.core.storage.storage import get_storage_id_for_state

    # check state folder exists
    folder_path = os.path.join(parent_path, get_storage_id_for_state(state))
    check_folder(folder_path, "state_path", missing_elements, existing_elements)

    # check state script exists
    if isinstance(state, ExecutionState):
        file_path = os.path.join(parent_path, get_storage_id_for_state(state), storage.SCRIPT_FILE)
        check_file(file_path, "script", missing_elements, existing_elements)

    # check state-meta data exists (transitions and so on)
    file_path = os.path.join(parent_path, get_storage_id_for_state(state), storage.FILE_NAME_CORE_DATA)
    check_file(file_path, "core data", missing_elements, existing_elements)

    # check if optional gui-meta-data exists
    if check_meta_data:
        # gtk gui meta data
        file_path = os.path.join(parent_path, get_storage_id_for_state(state), storage.FILE_NAME_META_DATA)
        check_file(file_path, "meta data", missing_elements, existing_elements)

    if isinstance(state, ContainerState):
        for key, child_state in state.states.iteritems():
            check_state_storage(child_state, folder_path, missing_elements, existing_elements, check_meta_data)


def check_that_all_files_are_there(state_machine, base_path=None, check_meta_data=False, with_print=False,
                                   old_exists=None, old_base_path=None):
    root_state = state_machine.root_state
    base_path = state_machine.file_system_path
    missing_elements = []
    existing_elements = []
    check_state_machine_storage(state_machine, base_path, missing_elements, existing_elements, check_meta_data)

    if old_exists is not None and old_base_path:
        old_without_base = [old_path.replace(old_base_path, "") for old_path in old_exists]
    else:
        old_without_base = None
    if with_print and missing_elements:
        logger.debug(30*"#")
        logger.debug("State machine %s with root_state.state_id %s MISSING the following FILES" % \
              (state_machine.state_machine_id, root_state.state_id))
        logger.debug(30*"#")
        for path in missing_elements:
            if old_without_base is not None and path.replace(base_path, "") in old_without_base:
                logger.debug(path, " ... but exists in ", old_base_path)
            else:
                logger.debug(path, " ... but does not exist before")
    else:
        logger.debug(30*"#")
        logger.debug("All Files and Folders where Found of state machine %s with root_state.state_id %s" % \
              (state_machine.state_machine_id, root_state.state_id))
        logger.debug(30*"#")

    assert len(missing_elements) == 0


def check_id_and_name_plus_id_format(path_old_format, path_new_format, sm_m):
    from rafcon.core.states.container_state import ContainerState

    def check_state(state, state_id=None, state_machine=False):
        # handle handed state machine
        if state_machine:
            folder = state.file_system_path # state is a state machine
            current_state = state.root_state
            state_id = state.root_state.state_id
        else:
            if state_id is None:
                raise
            folder = os.path.join(path_new_format, state.get_storage_path())
            current_state = state.states[state_id]

        # check that there is exact one child state folder with this state_id in this folder
        elements_found = [elem for elem in os.listdir(folder) if state_id in elem]
        if len(elements_found) > 1:
            logger.warning("Too many folders {2} for state_id: {0} in folder {1}".format(state_id, folder,
                                                                                         elements_found))
        elif len(elements_found) < 1:
            logger.warning("Too less folders {2} for state_id: {0} in folder {1}".format(state_id, folder,
                                                                                         elements_found))

        # recursive check of child states
        if isinstance(current_state, ContainerState):
            for state_id in current_state.states:
                check_state(current_state, state_id)

    check_state(sm_m.state_machine, state_machine=True)


@pytest.mark.parametrize("with_gui", [True, False])
def test_storage_with_gui(with_gui, caplog):
    print "test storage with gui", with_gui

    testing_utils.dummy_gui(None)

    if with_gui:
        testing_utils.run_gui(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False})
    else:
        testing_utils.initialize_environment(gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False},
                                             gui_already_started=False)

    e = None
    try:
        save_state_machine(with_gui)
    except Exception as e:
        pass
    finally:
        if with_gui:
            testing_utils.close_gui()
            testing_utils.shutdown_environment(caplog=caplog)
        else:
            testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)

    if e:
        raise e
    print "test storage with gui {0} finished".format(with_gui)


# TODO add examples of bad naming that cause before problems \n or [ ] and so on
def check_state_recursively_if_state_scripts_are_valid(state):
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.execution_state import ExecutionState
    if isinstance(state, ContainerState):
        for child_state in state.states.itervalues():
            check_state_recursively_if_state_scripts_are_valid(child_state)
    else:
        if isinstance(state, ExecutionState):
            assert os.path.exists(state.script.path) and state.script.script is not None


def test_on_clean_storing_with_name_in_path(caplog):
    print "test_on_clean_storing_with_name_in_path"

    testing_utils.dummy_gui(None)

    testing_utils.initialize_environment(
        gui_config={'HISTORY_ENABLED': False, 'AUTO_BACKUP_ENABLED': False}, gui_already_started=False)

    path_old_format = testing_utils.get_test_sm_path(
        os.path.join("unit_test_state_machines", "id_to_name_plus_id_storage_format_test_do_not_update"))
    path_new_format = os.path.join(testing_utils.get_unique_temp_path(),
                                   "id_to_name_plus_id_storage_format_test_do_not_update")

    # gui imports better after initialization
    from rafcon.gui.models.state_machine import StateMachineModel

    shutil.copytree(path_old_format, path_new_format)
    from rafcon.core.storage import storage
    sm = storage.load_state_machine_from_path(path_new_format)
    check_state_recursively_if_state_scripts_are_valid(sm.root_state)
    sm.base_path = path_new_format
    sm_m = StateMachineModel(sm)
    try:
        on_save_activate(sm_m, logger)
        check_that_all_files_are_there(sm, with_print=False)
        check_id_and_name_plus_id_format(path_old_format, path_new_format, sm_m)
    finally:
        testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


if __name__ == '__main__':
    # test_storage_with_gui(with_gui=True, caplog=None)
    # test_storage_with_gui(with_gui=False, caplog=None)
    # test_on_clean_storing_with_name_in_path(None)
    import pytest
    pytest.main(['-s', __file__])
