"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the state machine

.. moduleauthor:: Sebastian Brunner


"""

import sys
import argparse

from rafcon.statemachine.global_variable_manager import GlobalVariableManager
from rafcon.statemachine.library_manager import LibraryManager
from rafcon.statemachine.execution.state_machine_execution_engine import StateMachineExecutionEngine
from rafcon.statemachine.state_machine_manager import StateMachineManager
from rafcon.statemachine.enums import StateMachineExecutionStatus
from rafcon.statemachine.config import global_config


def signal_handler(signal, frame):
    # in this case the print is on purpose the see more easily if the interrupt signal reached the thread
    try:
        print 'SIGINT received! Execution engine will be stopped and program will be shutdown!'
        if state_machine_execution_engine.status.execution_mode is not StateMachineExecutionStatus.STOPPED:
            state_machine_execution_engine.stop()
            active_state_machine_id = state_machine_execution_engine.state_machine_manager.active_state_machine_id
            state_machine_execution_engine.state_machine_manager.state_machines[active_state_machine_id].root_state.join()
    except Exception as e:
        import traceback
        print "Could not stop statemachine: {0} {1}".format(e.message, traceback.format_exc())

    # shutdown twisted correctly
    try:
        # check if monitoring plugin is loaded
        from plugins.monitoring.monitoring_manager import global_monitoring_manager
        from twisted.internet import reactor
        print "Shutting down monitoring manager"
        global_monitoring_manager.shutdown()
        print "Shutting down twisted"
        if reactor.running:
            reactor.callFromThread(reactor.stop)
    except ImportError, e:
        # plugin not found
        pass

    # import sys
    # sys.exit(0)

    # this is a ugly process shutdown method, but works if gtk process are blocking
    import os
    os._exit(0)


# This variable holds the global variable manager singleton
global_variable_manager = GlobalVariableManager()

# This variable holds the library manager singleton
library_manager = LibraryManager()

# This variable holds the global state machine manager object
state_machine_manager = StateMachineManager()

# This variable holds the execution engine singleton
state_machine_execution_engine = StateMachineExecutionEngine(state_machine_manager)


argument_parser = argparse.ArgumentParser(description='Start RAFCON', fromfile_prefix_chars='@')