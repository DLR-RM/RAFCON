import sys
import logging
import gtk
import threading
import time
import glib
import os
import signal

from rafcon.utils import log
from rafcon.mvc.models import ContainerStateModel, StateModel, GlobalVariableManagerModel
from rafcon.mvc.controllers import MainWindowController, StateDataPortEditorController,\
    SingleWidgetWindowController, SourceEditorController
from rafcon.mvc.views.main_window import MainWindowView
from rafcon.mvc.views import LoggingView, StateDataportEditorView, SingleWidgetWindowView, SourceEditorView
from rafcon.mvc.models.state_machine_manager import StateMachineManagerModel
from rafcon.statemachine.states.hierarchy_state import HierarchyState
from rafcon.statemachine.states.execution_state import ExecutionState
import rafcon.mvc.singleton
from rafcon.statemachine.state_machine import StateMachine
import variables_for_pytest


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)

    global_var_manager_model = GlobalVariableManagerModel()

    return logger, global_var_manager_model


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)

def test_backward_stepping():

    variables_for_pytest.test_multithrading_lock.acquire()
    rafcon.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir("../rafcon/mvc/")
    signal.signal(signal.SIGINT, rafcon.statemachine.singleton.signal_handler)

    logger, gvm_model = create_models()

    rafcon.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = rafcon.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/return_none_test_sm")

    rafcon.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if variables_for_pytest.sm_manager_model is None:
        variables_for_pytest.sm_manager_model = rafcon.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    variables_for_pytest.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    rafcon.statemachine.singleton.state_machine_execution_engine.start()
    state_machine.root_state.join()
    rafcon.statemachine.singleton.state_machine_execution_engine.stop()

    assert state_machine.root_state.final_outcome.outcome_id == 0

    logger.debug("Gtk main loop exited!")
    os.chdir("../../test")
    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_backward_stepping()