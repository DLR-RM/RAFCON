
from rafcon.mvc.controllers.menu_bar import MenuBarController
from rafcon.statemachine.singleton import state_machine_execution_engine
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.statemachine.execution.statemachine_status import StateMachineStatus

from rafcon.utils import log
logger = log.get_logger(__name__)


class MonitoringMenuBar(MenuBarController):

    def __init__(self, state_machine_manager_model, view, shortcut_manager):
        MenuBarController.__init__(self, state_machine_manager_model, view, shortcut_manager)

    # overwrite all execution button functions
    def on_start_activate(self, widget, data=None):
        logger.info("Starting state machine on remote server ...")
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.STARTED)

    def on_start_from_selected_state_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.STARTED)

    def on_pause_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.PAUSED)
        logger.info("Pausing state machine on remote server ...")

    def on_stop_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.STOPPED)
        logger.info("Stopping state machine on remote server ...")

    def on_step_mode_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)

    def on_step_into_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.FORWARD_INTO)

    def on_step_over_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.FORWARD_OVER)

    def on_step_out_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.FORWARD_OUT)

    def on_backward_step_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.BACKWARD)

    def on_run_to_selected_state_activate(self, widget, data=None):
        state_machine_execution_engine.set_execution_mode(StateMachineExecutionStatus.RUN_TO_SELECTED_STATE)