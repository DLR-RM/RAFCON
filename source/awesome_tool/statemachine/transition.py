"""
.. module:: transition
   :platform: Unix, Windows
   :synopsis: A module to represent a transition in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
from utils import log
logger = log.get_logger(__name__)
import numpy


class Transition(Observable):
    """A class for representing a transition in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _from_state: the source state of the transition
    :ivar _from_outcome: the outcome of the source state
    :ivar _to_state: the target state of the transition
    :ivar _to_outcome: the outcome of the target state


    """

    def __init__(self, from_state=None, from_outcome=None, to_state=None, to_outcome=None):
        Observable.__init__(self)

        self._from_state = from_state
        self._from_outcome = from_outcome
        self._to_state = to_state
        self._to_outcome = to_outcome

        logger.debug("Transition with from state %s and from outcome %s initialized" %
                     (self._from_state, self._from_outcome))



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
            raise TypeError("ID must be of type numpy.uint32")

        self._from_state = from_state

    @property
    def from_outcome(self):
        """Property for the _from_outcome field

        """
        return self._from_outcome

    @from_outcome.setter
    @Observable.observed
    def from_outcome(self, from_outcome):
        if not isinstance(from_outcome, int):
            raise TypeError("ID must be of type int")

        self._from_outcome = from_outcome

    @property
    def to_state(self):
        """Property for the _to_state field

        """
        return self._to_state

    @to_state.setter
    @Observable.observed
    def to_state(self, to_state):
        if not isinstance(to_state, numpy.uint32):
            raise TypeError("ID must be of type numpy.uint32")

        self._to_state = to_state

    @property
    def to_outcome(self):
        """Property for the to_outcome field

        """
        return self._to_outcome

    @to_outcome.setter
    @Observable.observed
    def to_outcome(self, to_outcome):
        if not isinstance(to_outcome, int):
            raise TypeError("ID must be of type int")

        self._to_outcome = to_outcome
