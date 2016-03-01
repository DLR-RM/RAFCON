"""
.. module:: state_machine
   :platform: Unix, Windows
   :synopsis: A module to organize a state machine with all its main components

.. moduleauthor:: Rico Belder


"""

from gtkmvc import Observable

from rafcon.statemachine.id_generator import generate_state_machine_id
from rafcon.statemachine.execution.execution_history import ExecutionHistory
from rafcon.statemachine.enums import StateExecutionState

from rafcon.utils import log
logger = log.get_logger(__name__)


class StateMachine(Observable):
    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar state_machine_id: the id of the state machine
    :ivar root_state: the root state of the state machine
    :ivar base_path: the path, where to save the state machine

    """

    old_marked_dirty = True

    _root_state = None
    _marked_dirty = True
    _file_system_path = None

    def __init__(self, root_state=None):
        Observable.__init__(self)

        self.state_machine_id = generate_state_machine_id()
        self.root_state = root_state
        self.execution_history = ExecutionHistory()

    def start(self):
        """
        Starts the execution of the root state.
        :return:
        """
        # load default input data for the state
        self._root_state.input_data = self._root_state.get_default_input_values_for_state(self._root_state)
        self._root_state.output_data = self._root_state.create_output_dictionary_for_state(self._root_state)
        self._root_state.start(self.execution_history)

    def join(self):
        """Wait for root state to finish execution"""
        self._root_state.join()
        self._root_state.state_execution_status = StateExecutionState.INACTIVE

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def root_state(self):
        """Property for the _root_state field

        """
        return self._root_state

    @root_state.setter
    @Observable.observed
    def root_state(self, root_state):
        # if not isinstance(root_state, State):
        #     raise AttributeError("root_state has to be of type State")
        if root_state is not None:
            from rafcon.statemachine.states.state import State
            if not isinstance(root_state, State):
                raise AttributeError("root_state has to be of type State")
            root_state.parent = self
        self._root_state = root_state

    @Observable.observed
    def change_root_state_type(self, new_state_class):
        from rafcon.mvc.statemachine_helper import create_new_state_from_state_with_type

        state = self.root_state
        state_id = state.state_id

        new_state = create_new_state_from_state_with_type(state, new_state_class)
        assert new_state.state_id == state_id
        self.root_state = new_state  # Also sets the parent of root_state to self

        return new_state

    @property
    def marked_dirty(self):
        """Property for the _marked_dirty field

        """
        return self._marked_dirty

    @marked_dirty.setter
    @Observable.observed
    def marked_dirty(self, marked_dirty):
        # print "sm-core: marked dirty changed from ", self._marked_dirty, " to ", marked_dirty
        if not isinstance(marked_dirty, bool):
            raise AttributeError("marked_dirty has to be of type bool")
        self.old_marked_dirty = self._marked_dirty
        self._marked_dirty = marked_dirty

    def get_state_by_path(self, path, as_check=False):
        from rafcon.statemachine.states.library_state import LibraryState
        from rafcon.statemachine.states.execution_state import ExecutionState
        path_item_list = path.split('/')
        prev_state_id = path_item_list.pop(0)
        assert prev_state_id == self.root_state.state_id
        state = self.root_state
        for state_id in path_item_list:
            if isinstance(state, LibraryState):
                state = state.state_copy
                assert state.state_id == state_id
            else:
                if not isinstance(state, ExecutionState) and state_id in state.states:
                    state = state.states[state_id]
                else:
                    note = ''
                    if isinstance(state, ExecutionState):
                        note = ", hierarchy-level ends at ExecutionState '{0}'".format(prev_state_id)
                    if not as_check:
                        logger.warning("Invalid path '{0}' for state machine '{1}'{2}".format(path,
                                                                                              self.state_machine_id,
                                                                                              note))
                    return None
            prev_state_id = state_id
        return state

    @property
    def file_system_path(self):
        """Property for the _file_system_path field

        """
        return self._file_system_path

    @file_system_path.setter
    @Observable.observed
    def file_system_path(self, file_system_path):
        if not isinstance(file_system_path, basestring):
            raise AttributeError("file_system_path has to be of type str")
        self._file_system_path = file_system_path
