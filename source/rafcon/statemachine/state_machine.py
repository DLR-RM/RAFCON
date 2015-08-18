"""
.. module:: state_machine
   :platform: Unix, Windows
   :synopsis: A module to organize a state machine with all its main components

.. moduleauthor:: Rico Belder


"""

from gtkmvc import ModelMT, Observable
import threading

from rafcon.statemachine.id_generator import generate_state_machine_id
from rafcon.utils import log
logger = log.get_logger(__name__)
from rafcon.statemachine.execution.execution_history import ExecutionHistory
from rafcon.statemachine.enums import StateExecutionState


class StateMachine(ModelMT, Observable):

    """A class for to organizing all main components of a state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar state_machine_id: the id of the state machine
    :ivar root_state: the root state of the state machine
    :ivar base_path: the path, where to save the state machine

    """

    state_machine = None

    __observables__ = ("state_machine",)

    def __init__(self, root_state=None):
        ModelMT.__init__(self)
        Observable.__init__(self)

        # if root_state is not None:
        #     assert isinstance(root_state, State)

        self.state_machine = self

        self.state_machine_id = generate_state_machine_id()
        self._root_state = None
        self.root_state = root_state
        self._marked_dirty = True
        self.old_marked_dirty = True
        self.execution_history = ExecutionHistory()
        self._file_system_path = None

    def start(self):
        """
        Starts the execution of the root state.
        :return:
        """
        # load default input data for the state
        self._root_state.input_data = self._root_state.get_default_input_values_for_state(self._root_state)
        self._root_state.output_data = self._root_state.create_output_dictionary_for_state(self._root_state)
        self._root_state.start(self.execution_history)

        wait_for_finishing_thread = threading.Thread(target=self.wait_for_finishing, args=(self._root_state,))
        wait_for_finishing_thread.start()

    @staticmethod
    def wait_for_finishing(state):
        """ This method waits until a specific states finished its execution and stops the execution engine afterwards.

        :param state: the state to wait for its execution to finish
        :return:
        """
        state.join()
        state.state_execution_status = StateExecutionState.INACTIVE
        logger.debug("The final outcome of the state was %s" % (str(state.final_outcome)))
        # deferred import to avoid cyclic import at the beginning of the script
        from rafcon.statemachine.singleton import state_machine_execution_engine
        state_machine_execution_engine.stop()

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
    def change_root_state_type(self, state_m, new_state_class):
        from rafcon.mvc.statemachine_helper import StateMachineHelper
        return StateMachineHelper.change_state_type(state_m, new_state_class)

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

    def get_state_by_path(self, path):
        from rafcon.statemachine.states.library_state import LibraryState
        path_item_list = path.split('/')

        assert path_item_list.pop(0) == self.root_state.state_id
        state = self.root_state
        for state_id in path_item_list:
            if isinstance(state, LibraryState):
                state = state.state_copy
                assert state.state_id == state_id
            else:
                try:
                    state = state.states[state_id]
                except KeyError:
                    logger.warning("Invalid path '{0}' for state machine '{1}'".format(path, self))
                    return None
        return state

    @Observable.observed
    #def root_state_before_change(self, model, prop_name, instance, method_name, args, kwargs):
    def root_state_before_change(self, **kwargs):
        pass

    @Observable.observed
    def root_state_after_change(self, **kwargs):
    #def root_state_after_change(self, model, prop_name, instance, method_name, result, args, kwargs):
        pass

    @property
    def file_system_path(self):
        """Property for the _file_system_path field

        """
        return self._file_system_path

    @file_system_path.setter
    @Observable.observed
    def file_system_path(self, file_system_path):
        if not isinstance(file_system_path, str):
            raise AttributeError("file_system_path has to be of type str")
        self._file_system_path = file_system_path
