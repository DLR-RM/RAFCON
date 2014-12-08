"""
.. module:: transition
   :platform: Unix, Windows
   :synopsis: A module to represent a data flow connection in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import numpy


class DataFlow(Observable):

    """A class for representing a data flow connection in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _from_state: the source state of the data flow connection
    :ivar _from_key: the data key of the source state
    :ivar _to_state: the target state of the data flow connection
    :ivar _to_key: the data key of the target state

    """

    def __init__(self, from_state=None, from_key=None, to_state=None, to_key=None):

        Observable.__init__(self)

        self._from_state = from_state
        self._from_key = from_key
        self._to_state = to_state
        self._to_key = to_key


#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @Observable.observed
    def from_state(self, from_state):
        if not isinstance(from_state, numpy.uint32):
            raise TypeError("from_state must be of type numpy.uint32")

        self._from_state = from_state

    @property
    def from_key(self):
        """Property for the _from_key field

        """
        return self._from_key

    @from_key.setter
    @Observable.observed
    def from_key(self, from_key):
        if not isinstance(from_key, str):
            raise TypeError("from_key must be of type str")

        self._from_key = from_key

    @property
    def to_state(self):
        """Property for the _to_state field

        """
        return self._to_state

    @to_state.setter
    @Observable.observed
    def to_state(self, to_state):
        if not isinstance(to_state, numpy.uint32):
            raise TypeError("to_state must be of type numpy.uint32")

        self._to_state = to_state

    @property
    def to_key(self):
        """Property for the _to_key field

        """
        return self._to_key

    @to_key.setter
    @Observable.observed
    def to_key(self, to_key):
        if not isinstance(to_key, str):
            raise TypeError("to_key must be of type str")

        self._to_key = to_key

