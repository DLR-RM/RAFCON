"""
.. module:: singleton
   :synopsis: A module to hold all singletons of the state machine

"""

import sys
import argparse

from rafcon.core.global_variable_manager import GlobalVariableManager
from rafcon.core.library_manager import LibraryManager
from rafcon.core.execution.execution_engine import ExecutionEngine
from rafcon.core.state_machine_manager import StateMachineManager


# This variable holds the global variable manager singleton
global_variable_manager = GlobalVariableManager()

# This variable holds the library manager singleton
library_manager = LibraryManager()

# This variable holds the global state machine manager object
state_machine_manager = StateMachineManager()

# This variable holds the execution engine singleton
state_machine_execution_engine = ExecutionEngine(state_machine_manager)

# signal that cause shut down
shut_down_signal = None

argument_parser = argparse.ArgumentParser(description='Start RAFCON', fromfile_prefix_chars='@')
