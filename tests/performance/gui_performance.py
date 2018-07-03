# local
from rafcon.utils.timer import measure_time
import testing_utils

# general tool elements
from rafcon.utils import profiler
from rafcon.utils import log
logger = log.get_logger(__name__)


@measure_time
def add_state_machine_to_manager_model(state_machine, profiling=False):
    def _add_state_machine_to_manager_model():
        from rafcon.gui.singleton import state_machine_manager
        if profiling:
            profiler.start("add_state_machine")
        state_machine_manager.add_state_machine(state_machine)
        testing_utils.wait_for_gui()
        if profiling:
            profiler.stop("add_state_machine")
    testing_utils.call_gui_callback(_add_state_machine_to_manager_model)


@measure_time
def create_gui():
    testing_utils.run_gui(gui_config={'ENABLE_CACHING': False}, patch_threading=False, timeout=60)


@measure_time
def destroy_gui(caplog):
    testing_utils.close_gui()
    testing_utils.shutdown_environment(caplog=caplog, unpatch_threading=False)


def test_gui(number_child_states=10, number_childs_per_child=10, barrier=False, sleep=False, profile_add_state=True,
    caplog=None):
    create_gui()
    from rafcon.core.state_machine import StateMachine
    from core_performance import create_hierarchy_state, create_barrier_concurrency_state
    try:
        if barrier:
            state_machine = StateMachine(create_barrier_concurrency_state(number_child_states, number_childs_per_child))
        else:
            state_machine = StateMachine(create_hierarchy_state(number_child_states, sleep=sleep))

        add_state_machine_to_manager_model(state_machine, profile_add_state)
    except Exception:
        raise
    finally:
        destroy_gui(caplog)


if __name__ == '__main__':

    # global_profiling = True
    global_profiling = False

    if global_profiling:
        profiler.start("global")

    # test_gui(10)  # around 1 second
    # test_gui(10, execute=True)  # around 1.5 seconds
    test_gui(10, profile_add_state=not global_profiling)  # around 10 seconds
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
    
    if global_profiling:
        profiler.stop("global")
