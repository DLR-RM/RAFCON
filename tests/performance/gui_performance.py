# system
import threading
import gtk
import sys

# local
from core_performance import create_hierarchy_state, create_barrier_concurrency_state, measure_time
import testing_utils

# gui elements
import rafcon.gui.config as gui_config
import rafcon.gui.singleton
from rafcon.gui.controllers.main_window import MainWindowController
from rafcon.gui.views.main_window import MainWindowView

# core elements
import rafcon.core.config
from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.state_machine import StateMachine
import rafcon.core.singleton
from rafcon.core.config import global_config


# general tool elements
from rafcon.utils import profiler
from rafcon.utils import log
logger = log.get_logger(__name__)


def trigger_gui_signals(*args):
    sm_manager_model = args[0]
    main_window_controller = args[1]
    execute = args[2]
    menubar_ctrl = main_window_controller.get_controller('menu_bar_controller')
    # import time
    # time.sleep(5)
    testing_utils.wait_for_gui()
    rafcon.core.singleton.state_machine_execution_engine.start()
    rafcon.core.singleton.state_machine_execution_engine.join()
    testing_utils.call_gui_callback(menubar_ctrl.on_quit_activate, None, None, True)

@measure_time
def add_state_machine_to_manager_model(state_machine):
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()

@measure_time
def create_main_window():
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model
    main_window_view = MainWindowView()
    main_window_controller = MainWindowController(sm_manager_model, main_window_view)

    # Wait for GUI to initialize
    testing_utils.wait_for_gui()
    return main_window_controller

@measure_time
def test_gui(number_child_states=10, number_childs_per_child=10, barrier=False, execute=False, sleep=False):
    gui_config.global_gui_config.set_config_value('HISTORY_ENABLED', False)
    gui_config.global_gui_config.set_config_value('AUTO_BACKUP_ENABLED', False)
    gui_config.global_gui_config.set_config_value('GAPHAS_EDITOR', True)
    gui_config.global_gui_config.set_config_value('ENABLE_CACHING', False)
    rafcon.core.singleton.library_manager.refresh_libraries()

    if barrier:
        state_machine = StateMachine(create_barrier_concurrency_state(number_child_states, number_childs_per_child))
    else:
        state_machine = StateMachine(create_hierarchy_state(number_child_states, sleep=sleep))

    main_window_controller = create_main_window()

    add_state_machine_to_manager_model(state_machine)

    thread = threading.Thread(target=trigger_gui_signals, args=[rafcon.gui.singleton.state_machine_manager_model,
                                                                main_window_controller,
                                                                execute])
    thread.start()
    gtk.main()
    logger.debug("after gtk main")
    thread.join()

if __name__ == '__main__':

    enable_profiling = False

    if enable_profiling:
        profiler.start("global")
    # the recursion limit default is 1000
    # sys.setrecursionlimit(1000)
    # test_gui(10)  # around 1 second
    # test_gui(10, execute=True)  # around 1.5 seconds
    test_gui(20)  # around 10 seconds
    # TODO: executing states to fast without sleep or too less sleep leads to a recursion error
    # test_gui(100, execute=True, sleep=False)  # smallest state machine leading to recursion error
    # test_gui(50, execute=True, sleep=False)  # leads to recursion error
    # test_gui(100, execute=True, sleep=False)  # leads to recursion error
    # test_gui(80, execute=True, sleep=True)  # around 23.5 seconds
    # test_gui(100, execute=True, sleep=True)  # leads to recursion error with a sleep of 0.2 seconds per state
    # test_gui(200)  # around 48 seconds
    # test_gui(10, 10, True)  # around 5.8 seconds
    # test_gui(20, 10, True)  # around 24.5 seconds
    # test_gui(30, 10, True)  # around 57.5 seconds
    # test_gui(10, 20, True)  # around 19.6 seconds
    # test_gui(10, 30, True)  # around 40.6 seconds

    if enable_profiling:
        result_path = "/tmp"
        profiler.stop("global", result_path, True)
