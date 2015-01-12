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
import yaml


class Transition(Observable, yaml.YAMLObject):
    """A class for representing a transition in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _from_state: the source state of the transition
    :ivar _from_outcome: the outcome of the source state
    :ivar _to_state: the target state of the transition
    :ivar _to_outcome: the outcome of the target state


    """

    yaml_tag = u'!Transition'

    def __init__(self, from_state=None, from_outcome=None, to_state=None, to_outcome=None):
        Observable.__init__(self)

        self._from_state = None
        self.from_state = from_state

        self._from_outcome = None
        self.from_outcome = from_outcome

        self._to_state = None
        self.to_state = to_state

        self._to_outcome = None
        self.to_outcome = to_outcome

        logger.debug(self.__str__())

    def __str__(self):
        return "Transition - from_state: %s, from_outcome: %s, to_state: %s, to_outcome: %s" % (self._from_state,
                                                                                       self._from_outcome,
                                                                                       self._to_state, self._to_outcome)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'from_state': data.from_state,
            'from_outcome': data.from_outcome,
            'to_state': data.to_state,
            'to_outcome': data.to_outcome
        }
        print dict_representation
        node = dumper.represent_mapping(u'!Transition', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        from_state = dict_representation['from_state']
        from_outcome = dict_representation['from_outcome']
        to_state = dict_representation['to_state']
        to_outcome = dict_representation['to_outcome']
        return Transition(from_state, from_outcome, to_state, to_outcome)

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
        if not isinstance(from_state, str):
            raise TypeError("from_state must be of type str")

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
            raise TypeError("from_outcome must be of type int")

        self._from_outcome = from_outcome

    @property
    def to_state(self):
        """Property for the _to_state field

        """
        return self._to_state

    @to_state.setter
    @Observable.observed
    def to_state(self, to_state):
        if not to_state is None:
            if not isinstance(to_state, str):
                raise TypeError("to_state must be of type str")

        self._to_state = to_state

    @property
    def to_outcome(self):
        """Property for the to_outcome field

        """
        return self._to_outcome

    @to_outcome.setter
    @Observable.observed
    def to_outcome(self, to_outcome):
        if not to_outcome is None:
            if not isinstance(to_outcome, int):
                raise TypeError("to_outcome must be of type int")

        self._to_outcome = to_outcome
