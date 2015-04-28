import sys
import logging
import gtk
import os
import signal

from awesome_tool.utils import log
from awesome_tool.mvc.models import GlobalVariableManagerModel
from awesome_tool.mvc.views import LoggingView
import awesome_tool.mvc.singleton
import variables_for_pytest


def create_models():
    logger = log.get_logger(__name__)
    logger.setLevel(logging.DEBUG)
    for handler in logging.getLogger('gtkmvc').handlers:
        logging.getLogger('gtkmvc').removeHandler(handler)
    stdout = logging.StreamHandler(sys.stdout)
    stdout.setFormatter(logging.Formatter("%(asctime)s: %(levelname)-8s - %(name)s:  %(message)s"))
    stdout.setLevel(logging.DEBUG)
    logging.getLogger('gtkmvc').addHandler(stdout)
    logging.getLogger('statemachine.state').setLevel(logging.DEBUG)
    logging.getLogger('controllers.state_properties').setLevel(logging.DEBUG)

    global_var_manager_model = GlobalVariableManagerModel()
    global_var_manager_model.global_variable_manager.set_variable("global_variable_1", "value1")
    global_var_manager_model.global_variable_manager.set_variable("global_variable_2", "value2")

    return logger, global_var_manager_model


def setup_logger(logging_view):
    log.debug_filter.set_logging_test_view(logging_view)
    log.error_filter.set_logging_test_view(logging_view)


def test_error_propagation():

    variables_for_pytest.test_multithrading_lock.acquire()

    awesome_tool.statemachine.singleton.state_machine_manager.delete_all_state_machines()
    signal.signal(signal.SIGINT, awesome_tool.statemachine.singleton.signal_handler)
    logging_view = LoggingView()
    setup_logger(logging_view)

    awesome_tool.statemachine.singleton.library_manager.initialize()
    [state_machine, version, creation_time] = awesome_tool.statemachine.singleton.\
        global_storage.load_statemachine_from_yaml("../test_scripts/error_propagation_test")
    awesome_tool.statemachine.singleton.state_machine_manager.add_state_machine(state_machine)
    awesome_tool.statemachine.singleton.state_machine_execution_engine.start()
    sm = awesome_tool.statemachine.singleton.state_machine_manager.get_active_state_machine()
    if sm:
        sm.root_state.join()
    awesome_tool.statemachine.singleton.state_machine_execution_engine.stop()
    awesome_tool.statemachine.singleton.state_machine_manager.remove_state_machine(state_machine.state_machine_id)

    assert sm.root_state.output_data["error_check"] == "successfull"

    variables_for_pytest.test_multithrading_lock.release()


if __name__ == '__main__':
    test_error_propagation()