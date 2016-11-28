"""
.. module:: enums
   :platform: Unix, Windows
   :synopsis: A module which holds all global enumerations for the state machine

.. moduleauthor:: Sebastian Brunner


"""

from enum import Enum

from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE

StateType = Enum('STATE_TYPE', 'EXECUTION HIERARCHY BARRIER_CONCURRENCY PREEMPTION_CONCURRENCY LIBRARY DECIDER_STATE')
CallType = Enum('METHOD_NAME', 'EXECUTE CONTAINER')
StateExecutionStatus = Enum('STATE_EXECUTION_STATE', 'INACTIVE ACTIVE EXECUTE_CHILDREN WAIT_FOR_NEXT_STATE')
StateMachineExecutionStatus = Enum('STATE_MACHINE_EXECUTION_STATUS', 'STARTED STOPPED PAUSED '
                                                                     'FORWARD_INTO FORWARD_OVER FORWARD_OUT '
                                                                     'BACKWARD RUN_TO_SELECTED_STATE')

# Constants
UNIQUE_DECIDER_STATE_ID = "unique_decider_state_id"
DEFAULT_SCRIPT_PATH = RAFCON_TEMP_PATH_STORAGE
