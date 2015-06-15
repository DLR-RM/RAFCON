import sys
import logging
import gtk
import threading
import time
import glib
import os
import signal

from awesome_tool.utils import log
from awesome_tool.mvc.models import ContainerStateModel, StateModel, GlobalVariableManagerModel
from awesome_tool.mvc.controllers import MainWindowController, StateDataPortEditorController,\
    SingleWidgetWindowController, SourceEditorController
from awesome_tool.mvc.views.main_window import MainWindowView
from awesome_tool.mvc.views import LoggingView, StateDataportEditorView, SingleWidgetWindowView, SourceEditorView
from awesome_tool.mvc.models.state_machine_manager import StateMachineManagerModel
from awesome_tool.statemachine.states.hierarchy_state import HierarchyState
from awesome_tool.statemachine.states.execution_state import ExecutionState
import awesome_tool.mvc.singleton
from awesome_tool.statemachine.state_machine import StateMachine
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
    awesome_tool.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    os.chdir("../awesome_tool/mvc/")
    signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)

    logger, gvm_model = create_models()

    awesome_tool.statemachine.singleton.library_manager.initialize()

    [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../../test_scripts/return_none_test_sm")

    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    if variables_for_pytest.sm_manager_model is None:
        variables_for_pytest.sm_manager_model = awesome_tool.mvc.singleton.state_machine_manager_model

    # load the meta data for the state machine
    variables_for_pytest.sm_manager_model.get_selected_state_machine_model().root_state.load_meta_data_for_state()

    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    state_machine.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()

    assert state_machine.root_state.final_outcome.outcome_id == 0

    logger.debug("Gtk main loop exited!")
    os.chdir("../../test")
    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_backward_stepping()