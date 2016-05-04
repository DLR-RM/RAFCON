"""
.. module:: state_helper
   :platform: Unix, Windows
   :synopsis: A module to represent a state in the statemachine

.. moduleauthor:: Sebastian Brunner


"""
import os

from rafcon.statemachine.storage import storage
from rafcon.utils.constants import RAFCON_TEMP_PATH_STORAGE


class StateHelper(object):

    """A class that holds utility functions to modify and handle states.

    """

    def __init__(self):
        pass

    @staticmethod
    def get_state_copy(source_state):
        """This functions returns a state copy of a given state.

        :param source_state: the source state to make a copy of
        :return: the copy of the source state
        """
        temporary_storage_path = RAFCON_TEMP_PATH_STORAGE+"/state_copy_tmp_folder"
        if not os.path.exists(temporary_storage_path):
            os.makedirs(temporary_storage_path)
        storage.save_state_recursively(source_state, temporary_storage_path, "")
        state_copy = storage.load_state_from_path(os.path.join(temporary_storage_path, source_state.state_id))

        from rafcon.statemachine.states.execution_state import ExecutionState
        from rafcon.statemachine.states.container_state import ContainerState

        def check_for_source_scripts(state_copy, source_state):

            if isinstance(state_copy, ExecutionState):
                state_copy.script_text = source_state.script_text
            if isinstance(state_copy, ContainerState):
                for state_id, state in state_copy.states.iteritems():
                    check_for_source_scripts(state, source_state.states[state_id])

        check_for_source_scripts(state_copy, source_state)
        state_copy.change_state_id()

        return state_copy
