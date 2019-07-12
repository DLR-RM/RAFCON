from builtins import str
import sys
import os
from gi.repository import Gtk
import signal
from os.path import dirname, abspath


from rafcon.core.states.hierarchy_state import HierarchyState
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.states.library_state import LibraryState
from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
from rafcon.core.state_machine import StateMachine
from rafcon.core.storage import storage

import rafcon.core.singleton
import rafcon.gui.singleton

from rafcon.core.config import global_config
from rafcon.gui.config import global_gui_config

from rafcon.utils import log
logger = log.get_logger(__name__)


def create_turtle_statemachine(base_path, example_path):
    basic_turtle_demo_state = HierarchyState("BasicTurtleDemo")
    init_ros_node = LibraryState("ros_libraries", "init_ros_node", "0.1", "init ros node")

    basic_turtle_demo_state.add_state(init_ros_node)
    basic_turtle_demo_state.set_start_state(init_ros_node.state_id)

    ########################################################
    # Turtle Concurrency State
    ########################################################

    preemptive_concurrency_state = PreemptiveConcurrencyState("Turtle Concurrency State")
    basic_turtle_demo_state.add_state(preemptive_concurrency_state)
    basic_turtle_demo_state.add_transition(init_ros_node.state_id, 0, preemptive_concurrency_state.state_id, None)
    basic_turtle_demo_state.add_transition(preemptive_concurrency_state.state_id, 0, basic_turtle_demo_state.state_id, 0)

    ########################################################
    # Subscribe to turtle position concurrency State
    ########################################################

    subscribe_to_turtle_position_hierarchy_state = HierarchyState("Turtle Position Subscriber Hierarchy State")

    preemptive_concurrency_state.add_state(subscribe_to_turtle_position_hierarchy_state)

    spawn_turtle = LibraryState("turtle_libraries", "turtle_position_subscriber", "0.1", "subscribe to turtle position")
    subscribe_to_turtle_position_hierarchy_state.add_state(spawn_turtle)
    subscribe_to_turtle_position_hierarchy_state.set_start_state(spawn_turtle.state_id)
    subscribe_to_turtle_position_hierarchy_state.add_transition(spawn_turtle.state_id, 0, spawn_turtle.state_id, None)

    ########################################################
    # Move Turtle Hierarchy State
    ########################################################
    move_turtle_hierarchy_state = HierarchyState("Move Turtle Hierarchy State")
    preemptive_concurrency_state.add_state(move_turtle_hierarchy_state)
    preemptive_concurrency_state.add_transition(move_turtle_hierarchy_state.state_id, 0,
                                                preemptive_concurrency_state.state_id, 0)

    spawn_turtle = LibraryState("turtle_libraries", "spawn_turtle", "0.1", "spawn turtle")
    move_turtle_hierarchy_state.add_state(spawn_turtle)
    move_turtle_hierarchy_state.set_start_state(spawn_turtle.state_id)

    wait1 = ExecutionState("Wait1", path=base_path, filename="wait.py")
    move_turtle_hierarchy_state.add_state(wait1)
    move_turtle_hierarchy_state.add_transition(spawn_turtle.state_id, 0, wait1.state_id, None)

    teleport_turtle = LibraryState("turtle_libraries", "teleport_turtle", "0.1", "teleport turtle")
    move_turtle_hierarchy_state.add_state(teleport_turtle)
    move_turtle_hierarchy_state.add_transition(wait1.state_id, 0, teleport_turtle.state_id, None)

    wait2 = ExecutionState("Wait2", path=base_path, filename="wait.py")
    move_turtle_hierarchy_state.add_state(wait2)
    move_turtle_hierarchy_state.add_transition(teleport_turtle.state_id, 0, wait2.state_id, None)

    clear_field = LibraryState("turtle_libraries", "clear_field", "0.1", "clear field")
    move_turtle_hierarchy_state.add_state(clear_field)
    move_turtle_hierarchy_state.add_transition(wait2.state_id, 0, clear_field.state_id, None)

    wait3 = LibraryState(name="Wait3", library_path="generic", library_name="wait")
    move_turtle_hierarchy_state.add_state(wait3)
    move_turtle_hierarchy_state.add_transition(clear_field.state_id, 0, wait3.state_id, None)

    set_velocity1 = LibraryState("turtle_libraries", "set_velocity", "0.1", "set velocity1")
    move_turtle_hierarchy_state.add_state(set_velocity1)
    move_turtle_hierarchy_state.add_transition(wait3.state_id, 0, set_velocity1.state_id, None)

    wait4 = ExecutionState("Wait4", path=base_path, filename="wait.py")
    move_turtle_hierarchy_state.add_state(wait4)
    move_turtle_hierarchy_state.add_transition(set_velocity1.state_id, 0, wait4.state_id, None)

    move_to_position = LibraryState("turtle_libraries", "move_to_position", "0.1", "move to position")
    move_turtle_hierarchy_state.add_state(move_to_position)
    move_turtle_hierarchy_state.add_transition(wait4.state_id, 0, move_to_position.state_id, None)

    move_turtle_hierarchy_state.add_transition(move_to_position.state_id, 1, move_to_position.state_id, None)

    kill_turtle = LibraryState("turtle_libraries", "kill_turtle", "0.1", "kill turtle")
    move_turtle_hierarchy_state.add_state(kill_turtle)
    move_turtle_hierarchy_state.add_transition(move_to_position.state_id, 0, kill_turtle.state_id, None)

    move_turtle_hierarchy_state.add_transition(kill_turtle.state_id, 0, move_turtle_hierarchy_state.state_id, 0)

    return basic_turtle_demo_state


def run_turtle_demo():
    import rafcon.core
    import rafcon.core.start
    from rafcon.utils.i18n import setup_l10n
    signal.signal(signal.SIGINT, rafcon.core.start.signal_handler)
    setup_l10n()
    global_config.load()
    global_gui_config.load()
    # set the test_libraries path temporarily to the correct value
    library_paths = rafcon.core.config.global_config.get_config_value("LIBRARY_PATHS")
    if os.path.exists(str(os.path.sep).join([rafcon.__path__[0], '..', '..', '..', 'share', 'rafcon', 'libraries'])):  # rm-pkg
        os.environ['RAFCON_LIB_PATH'] = os.path.join(rafcon.__path__[0], '..', '..', '..', 'share', 'rafcon', 'libraries')
        library_paths["ros_libraries"] = os.path.join(rafcon.__path__[0], '..', '..', '..',
                                                      'share', 'rafcon', 'examples', 'libraries', 'ros_libraries')
        library_paths["turtle_libraries"] = os.path.join(rafcon.__path__[0], '..', '..', '..',
                                                         'share', 'rafcon', 'examples', 'libraries', 'turtle_libraries')
        example_path = os.path.join(rafcon.__path__[0], os.pardir, '..', '..', 'share', 'examples', "tutorials")
    else:  # git repo
        os.environ['RAFCON_LIB_PATH'] = os.path.join(dirname(abspath(__file__)), '..', '..', '..', 'libraries')
        library_paths["ros_libraries"] = os.path.join(dirname(abspath(__file__)), '..', '..', 'libraries', 'ros_libraries')
        library_paths["turtle_libraries"] = os.path.join(dirname(abspath(__file__)), '..', '..', 'libraries', 'turtle_libraries')
        example_path = os.path.join(dirname(abspath(__file__)), '..', '..', "tutorials")
    rafcon.core.singleton.library_manager.initialize()
    rafcon.core.singleton.state_machine_manager.delete_all_state_machines()
    base_path = os.path.dirname(os.path.abspath(__file__))

    basic_turtle_demo_state = create_turtle_statemachine(base_path, example_path)
    state_machine = StateMachine(basic_turtle_demo_state)

    # # load the state machine
    # [state_machine, version, creation_time] = storage.load_statemachine_from_path(
    #     "../../share/examples/tutorials/basic_turtle_demo_sm")

    from rafcon.gui.controllers.main_window import MainWindowController
    from rafcon.gui.views.main_window import MainWindowView

    rafcon.core.singleton.library_manager.initialize()
    main_window_view = MainWindowView()
    rafcon.core.singleton.state_machine_manager.add_state_machine(state_machine)
    sm_manager_model = rafcon.gui.singleton.state_machine_manager_model

    main_window_controller = MainWindowController(sm_manager_model, main_window_view)

    Gtk.main()
    logger.debug("Gtk main loop exited!")


if __name__ == '__main__':
    cur_path = os.path.abspath(os.path.dirname(__file__))
    test_script_path = os.path.join(cur_path, os.pardir, os.pardir, 'test_scripts')
    sys.path.insert(1, test_script_path)
    run_turtle_demo()
