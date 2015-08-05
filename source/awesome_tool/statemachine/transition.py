"""
.. module:: transition
   :platform: Unix, Windows
   :synopsis: A module to represent a transition in the state machine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
from awesome_tool.utils import log
logger = log.get_logger(__name__)
import yaml

from awesome_tool.statemachine.id_generator import *


class Transition(Observable, yaml.YAMLObject):
    """A class for representing a transition in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar transition_id: the id of the transition
    :ivar _from_state: the source state of the transition
    :ivar _from_outcome: the outcome of the source state
    :ivar _to_state: the target state of the transition
    :ivar _to_outcome: the outcome of the target state


    """

    yaml_tag = u'!Transition'

    def __init__(self, from_state, from_outcome, to_state, to_outcome, transition_id, parent=None):
        Observable.__init__(self)

        self._transition_id = None
        if transition_id is None:
            self.transition_id = generate_transition_id()
        else:
            self.transition_id = transition_id

        self._from_state = None
        self.from_state = from_state

        self._from_outcome = None
        self.from_outcome = from_outcome

        self._to_state = None
        self.to_state = to_state

        # if to_state is None:
        #     raise TypeError("to_state must not be None")
        #     exit(-1)

        self._to_outcome = None
        self.to_outcome = to_outcome

        self._parent = None
        self.parent = parent

        logger.debug(self.__str__())

    def __str__(self):
        return "Transition - from_state: %s, from_outcome: %s, to_state: %s, to_outcome: %s, id: %s" %\
               (self._from_state, self._from_outcome, self._to_state, self._to_outcome, self._transition_id)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'transition_id': data.transition_id,
            'from_state': data.from_state,
            'from_outcome': data.from_outcome,
            'to_state': data.to_state,
            'to_outcome': data.to_outcome
        }
        node = dumper.represent_mapping(u'!Transition', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        transition_id = dict_representation['transition_id']
        from_state = dict_representation['from_state']
        from_outcome = dict_representation['from_outcome']
        to_state = dict_representation['to_state']
        to_outcome = dict_representation['to_outcome']
        return Transition(from_state, from_outcome, to_state, to_outcome, transition_id)

#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @Observable.observed
    def modify_origin(self, from_state, from_outcome):
        """ Set from_state and from_outcome at ones to support fully valid transition modifications.
        :param str from_state: valid origin state
        :param int from_outcome: valid origin outcome
        :return:
        """
        if not (from_state is None and from_outcome is None):
            if not isinstance(from_state, str):
                raise TypeError("from_state must be of type str")
            if not isinstance(from_outcome, int):
                raise TypeError("from_outcome must be of type int")

        self._from_state = from_state
        self._from_outcome = from_outcome

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    # @Observable.observed  # should not be observed to stay consistent
    def from_state(self, from_state):
        if from_state is not None and not isinstance(from_state, str):
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
        if from_outcome is not None and not isinstance(from_outcome, int):
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
        if to_state is not None and not isinstance(to_state, str):
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
        if to_outcome is not None and not isinstance(to_outcome, int):
            raise TypeError("to_outcome must be of type int")

        self._to_outcome = to_outcome

    @property
    def transition_id(self):
        """Property for the _transition_id field

        """
        return self._transition_id

    @transition_id.setter
    @Observable.observed
    def transition_id(self, transition_id):
        if not transition_id is None:
            if not isinstance(transition_id, int):
                raise TypeError("transition_id must be of type int")

        self._transition_id = transition_id

    @property
    def parent(self):
        return self._parent

    @parent.setter
    @Observable.observed
    def parent(self, parent):
        if parent is not None:
            from awesome_tool.statemachine.states.state import State
            assert isinstance(parent, State)
        self._parent = parent