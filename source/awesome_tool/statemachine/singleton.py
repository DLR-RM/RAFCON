"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.global_variable_manager import GlobalVariableManager
from statemachine.library_manager import LibraryManager
from statemachine.execution.statemachine_execution_engine import StatemachineExecutionEngine
from statemachine.storage.storage import Storage
from statemachine.state_machine_manager import StateMachineManager

#This variable holds the global variable manager singleton
global_variable_manager = GlobalVariableManager()

#This variable holds the library manager singleton
library_manager = LibraryManager()

#This variable holds the global state machine manager object
state_machine_manager = StateMachineManager()

#This variable holds the execution engine singleton
state_machine_execution_engine = StatemachineExecutionEngine(state_machine_manager)

#This variable holds a global storage object
global_storage = Storage("")