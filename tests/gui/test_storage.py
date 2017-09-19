import os
import gtk
import logging
import shutil
import pytest

# mvc
import rafcon.gui.singleton
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView
from rafcon.gui.models.state_machine import StateMachineModel

# core elements
import rafcon.core.singleton
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.container_state import ContainerState
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.state_machine import StateMachine
from rafcon.core.storage import storage

# general tool elements
from rafcon.utils import log

# test environment elements
import testing_utils
from testing_utils import call_gui_callback

logger = log.get_logger(__name__)


def create_models(*args, **kargs):

    logger.setLevel(logging.DEBUG)
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

    state_dict = {'Container': ctr_state, 'State1': state1, 'State2': state2, 'State3': state3, 'Nested': state4, 'Nested2': state5}
    sm = StateMachine(ctr_state)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

    for sm_in in rafcon.core.singleton.state_machine_manager.state_machines.values():
        rafcon.core.singleton.state_machine_manager.remove_state_machine(sm_in.state_machine_id)
    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)

    rafcon.core.singleton.state_machine_manager.add_state_machine(sm)
    rafcon.gui.singleton.state_machine_manager_model.selected_state_machine_id = sm.state_machine_id

    sm_m = rafcon.gui.singleton.state_machine_manager_model.state_machines[sm.state_machine_id]

    # return ctr_state, sm_m, state_dict
    return ctr_state, sm_m, state_dict


def on_save_activate(state_machine_m, logger):
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


def save_state_machine(sm_model, path, logger, with_gui, menubar_ctrl):
    # sleep_time_short = 0.5
    if with_gui:
        sm_model.state_machine.base_path = path
        # time.sleep(sleep_time_short)
        call_gui_callback(menubar_ctrl.on_save_as_activate, None, None, sm_model.state_machine.base_path)
        # time.sleep(sleep_time_short)
        call_gui_callback(menubar_ctrl.on_quit_activate, None)
    else:
        sm_model.state_machine.base_path = path
        on_save_activate(sm_model, logger)


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


def check_that_all_files_are_there(sm_m, base_path=None, check_meta_data=False, with_print=False,
                                   old_exists=None, old_base_path=None):
    root_state = sm_m.state_machine.root_state
    base_path = sm_m.state_machine.file_system_path
    missing_elements = []
    existing_elements = []
    check_state_machine_storage(sm_m.state_machine, base_path, missing_elements, existing_elements, check_meta_data)

    if old_exists is not None and old_base_path:
        old_without_base = [old_path.replace(old_base_path, "") for old_path in old_exists]
    else:
        old_without_base = None
    if with_print and missing_elements:
        logger.debug(30*"#")
        logger.debug("State machine %s with root_state.state_id %s MISSING the following FILES" % \
              (sm_m.state_machine.state_machine_id, root_state.state_id))
        logger.debug(30*"#")
        for path in missing_elements:
            if old_without_base is not None and path.replace(base_path, "") in old_without_base:
                logger.debug(path, " ... but exists in ", old_base_path)
            else:
                logger.debug(path, " ... but does not exist before")
    else:
        logger.debug(30*"#")
        logger.debug("All Files and Folders where Found of state machine %s with root_state.state_id %s" % \
              (sm_m.state_machine.state_machine_id, root_state.state_id))
        logger.debug(30*"#")

    return missing_elements, existing_elements


def check_id_and_name_plus_id_format(path_old_format, path_new_format, sm_m):

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


def test_storage_without_gui(caplog):
    with_gui = False
    testing_utils.initialize_environment()
    [state, sm_m, state_dict] = create_models()
    try:
        save_state_machine(sm_model=sm_m, path=testing_utils.get_unique_temp_path(), logger=logger, with_gui=with_gui,
                           menubar_ctrl=None)

        missing_elements, _ = check_that_all_files_are_there(sm_m, with_print=False)

        assert len(missing_elements) == 0
    finally:
        testing_utils.shutdown_environment(caplog=caplog)


def test_storage_with_gui(caplog):
    # with_gui = True
    # testing_utils.run_gui()
    # [state, sm_m, state_dict] = create_models()
    # menubar_ctrl = rafcon.gui.singleton.main_window_controller.get_controller('menu_bar_controller')
    # try:
    #     save_state_machine(sm_model=sm_m, path=testing_utils.get_unique_temp_path(), logger=logger, with_gui=with_gui,
    #                        menubar_ctrl=menubar_ctrl)
    # finally:
    #     menubar_ctrl.on_quit_activate(None, None, True)
    #
    # missing_elements, _ = check_that_all_files_are_there(sm_m, check_meta_data=True, with_print=False)
    # assert len(missing_elements) == 0
    # testing_utils.shutdown_environment()
    # testing_utils.assert_logger_warnings_and_errors(caplog)

    with_gui = True
    testing_utils.initialize_environment()
    logger.debug("create model")
    [state, sm_m, state_dict] = create_models()

    main_window_controller = MainWindowController(rafcon.gui.singleton.state_machine_manager_model, MainWindowView())

    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    testing_utils.wait_for_gui()

    logger.debug("start thread")
    import threading
    thread = threading.Thread(target=save_state_machine,
                              args=[sm_m, testing_utils.get_unique_temp_path(), logger, with_gui, menubar_ctrl])
    thread.start()

    if with_gui:
        gtk.main()
        logger.debug("Gtk main loop exited!")

    thread.join()

    missing_elements, _ = check_that_all_files_are_there(sm_m, check_meta_data=True, with_print=False)
    try:
        assert len(missing_elements) == 0
    finally:
        testing_utils.shutdown_environment(caplog=caplog)


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
    testing_utils.initialize_environment(gui_config={"AUTO_BACKUP_ENABLED": True})

    path_old_format = testing_utils.get_test_sm_path(
        os.path.join("unit_test_state_machines", "id_to_name_plus_id_storage_format_test_do_not_update"))
    path_new_format = os.path.join(testing_utils.get_unique_temp_path(),
                                   "id_to_name_plus_id_storage_format_test_do_not_update")
    shutil.copytree(path_old_format, path_new_format)
    sm = storage.load_state_machine_from_path(path_new_format)
    check_state_recursively_if_state_scripts_are_valid(sm.root_state)
    sm.base_path = path_new_format
    sm_m = StateMachineModel(sm)
    try:
        on_save_activate(sm_m, logger)

        missing_elements, _ = check_that_all_files_are_there(sm_m, with_print=False)

        assert len(missing_elements) == 0

        check_id_and_name_plus_id_format(path_old_format, path_new_format, sm_m)
    finally:
        testing_utils.shutdown_environment(caplog=caplog)


if __name__ == '__main__':
    test_storage_without_gui(None)
    test_storage_with_gui(None)
    test_on_clean_storing_with_name_in_path(None)
    # pytest.main([__file__])
