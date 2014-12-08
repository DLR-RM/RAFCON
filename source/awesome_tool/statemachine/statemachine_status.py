"""
.. module:: state_machine_status
   :platform: Unix, Windows
   :synopsis: A module to represent the state machine status

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

from execution_history import ExecutionHistory


class StateMachineStatus(Observable):

    """A class for representing the state machine status

    It inherits from Observable to make a change of its fields observable.

    :ivar _state_id: the id of the state
    :ivar _name: the name of the state
    :ivar _input_keys: holds the input data keys of the state
    :ivar _output_keys: holds the output data keys of the state


    """

    def __init__(self, status=None, is_stepping=None, is_forward_run=None):

        Observable.__init__(self)

        # these fields are not supposed to be written by the GUI directly, but via the methods os the
        # StateMachineExecutionEngine class
        self._status = status
        self._is_stepping = is_stepping
        self._is_forward_run = is_forward_run
        self._dependency_tree = None
        self._thread_histories = []

    def set_history(self, hid, execution_history):
        if not isinstance(execution_history, ExecutionHistory):
            raise TypeError("execution_history must be of type ExecutionHistory")
        self._thread_histories[hid] = execution_history


#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################


    @property
    def status(self):
        """Property for the _status field

        """
        return self._status

    @property
    def is_stepping(self):
        """Property for the _is_stepping field

        """
        return self._is_stepping

    @property
    def is_forward_run(self):
        """Property for the _is_forward_run field

        """
        return self._is_forward_run

    @property
    def dependency_tree(self):
        """Property for the _dependency_tree field

        """
        return self._dependency_tree

    @property
    def thread_histories(self):
        """Property for the _thread_histories field

        """
        return self._thread_histories