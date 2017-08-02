# state machine
from rafcon.core.states.state import State
from rafcon.core.decorators import global_lock_counter, lock_state_machine
from rafcon.core.states.execution_state import ExecutionState
from rafcon.core.state_machine import StateMachine

from testing_utils import assert_logger_warnings_and_errors
from rafcon.utils import log
logger = log.get_logger(__name__)


def test_lock_state_machine(caplog):

    state_machine = StateMachine()

    @lock_state_machine
    def custom_function(object, number):
        raise AttributeError("Test error")

    State.custom_method = custom_function

    state1 = ExecutionState("s1")
    state_machine.root_state = state1

    try:
        state1.custom_method(5)
    except Exception, e:
        import traceback
        print "Could not stop state machine: {0} {1}".format(e.message, traceback.format_exc())

    assert global_lock_counter == 0

    state1.add_outcome("outcome1", 3)
    assert len(state1.outcomes) == 4

    assert_logger_warnings_and_errors(caplog)


if __name__ == '__main__':
    test_lock_state_machine(None)
