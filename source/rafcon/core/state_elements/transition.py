# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: transition
   :synopsis: A module to represent a transition in the state machine

"""

from weakref import ref
from future.utils import string_types
from gtkmvc3.observable import Observable

from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config
from rafcon.utils import log
logger = log.get_logger(__name__)

from rafcon.core.id_generator import *


class Transition(StateElement):
    """A class for representing a transition in the state machine

    It inherits from Observable to make a change of its fields observable.

    Raises an exceptions.TypeError if the transition_id is not of type int.

    :ivar int transition_id: the id of the transition, must be unique for the parent state
    :ivar str Transition.from_state: the source state of the transition
    :ivar int Transition.from_outcome: the outcome of the source state
    :ivar str Transition.to_state: the target state of the transition
    :ivar int Transition.to_outcome: the outcome of the target state
    :ivar rafcon.core.states.container_state.ContainerState StateElement.parent: reference to the parent state
    """

    yaml_tag = u'!Transition'

    _transition_id = None
    _from_state = None
    _from_outcome = None
    _to_state = None
    _to_outcome = None

    def __init__(self, from_state, from_outcome, to_state, to_outcome, transition_id, parent=None, safe_init=True):
        super(Transition, self).__init__(safe_init=safe_init)

        if transition_id is None:
            self._transition_id = generate_transition_id()
        else:
            if not isinstance(transition_id, int):
                raise TypeError("transition_id must be of type int")
            self._transition_id = transition_id

            if safe_init:
                Transition._safe_init(self, from_state, from_outcome, to_state, to_outcome, parent)
            else:
                Transition._unsafe_init(self, from_state, from_outcome, to_state, to_outcome, parent)

    def _safe_init(self, from_state, from_outcome, to_state, to_outcome, parent):
        self.from_state = from_state
        self.from_outcome = from_outcome
        self.to_state = to_state
        self.to_outcome = to_outcome
        # Checks for validity
        self.parent = parent

    def _unsafe_init(self, from_state, from_outcome, to_state, to_outcome, parent):
        self._from_state = from_state
        self._from_outcome = from_outcome
        self._to_state = to_state
        self._to_outcome = to_outcome
        if parent:
            self._parent = ref(parent)

    def __str__(self):
        return "Transition - from_state: %s, from_outcome: %s, to_state: %s, to_outcome: %s, id: %s" %\
               (self._from_state, self._from_outcome, self._to_state, self._to_outcome, self._transition_id)

    def __copy__(self):
        return self.__class__(self._from_state, self._from_outcome, self._to_state, self._to_outcome,
                              self._transition_id, None, safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        return self._transition_id

    @classmethod
    def from_dict(cls, dictionary):
        transition_id = dictionary['transition_id']
        from_state = dictionary['from_state']
        from_outcome = dictionary['from_outcome']
        to_state = dictionary['to_state']
        to_outcome = dictionary['to_outcome']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return cls(from_state, from_outcome, to_state, to_outcome, transition_id, safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'transition_id': state_element.transition_id,
            'from_state': state_element.from_state,
            'from_outcome': state_element.from_outcome,
            'to_state': state_element.to_state,
            'to_outcome': state_element.to_outcome
        }

#########################################################################
# Properties for all class field that must be observed by the gtkmvc3
#########################################################################

    @lock_state_machine
    @Observable.observed
    def modify_origin(self, from_state, from_outcome):
        """Set both from_state and from_outcome at the same time to modify transition origin

        :param str from_state: State id of the origin state
        :param int from_outcome: Outcome id of the origin port
        :raises exceptions.ValueError: If parameters have wrong types or the new transition is not valid
        """
        if not (from_state is None and from_outcome is None):
            if not isinstance(from_state, string_types):
                raise ValueError("Invalid transition origin port: from_state must be a string")
            if not isinstance(from_outcome, int):
                raise ValueError("Invalid transition origin port: from_outcome must be of type int")

        old_from_state = self.from_state
        old_from_outcome = self.from_outcome
        self._from_state = from_state
        self._from_outcome = from_outcome

        valid, message = self._check_validity()
        if not valid:
            self._from_state = old_from_state
            self._from_outcome = old_from_outcome
            raise ValueError("The transition origin could not be changed: {0}".format(message))

    @lock_state_machine
    @Observable.observed
    def modify_target(self, to_state, to_outcome=None):
        """Set both to_state and to_outcome at the same time to modify transition target

        :param str to_state: State id of the target state
        :param int to_outcome: Outcome id of the target port
        :raises exceptions.ValueError: If parameters have wrong types or the new transition is not valid
        """
        if not (to_state is None and (to_outcome is not int and to_outcome is not None)):
            if not isinstance(to_state, string_types):
                raise ValueError("Invalid transition target port: to_state must be a string")
            if not isinstance(to_outcome, int) and to_outcome is not None:
                raise ValueError("Invalid transition target port: to_outcome must be of type int or None (if to_state "
                                 "is of type str)")

        old_to_state = self.to_state
        old_to_outcome = self.to_outcome
        self._to_state = to_state
        self._to_outcome = to_outcome

        valid, message = self._check_validity()
        if not valid:
            self._to_state = old_to_state
            self._to_outcome = old_to_outcome
            raise ValueError("The transition target could not be changed: {0}".format(message))

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @lock_state_machine
    # @Observable.observed  # should not be observed to stay consistent
    def from_state(self, from_state):
        if from_state is not None and not isinstance(from_state, string_types):
            raise ValueError("from_state must be a string")

        self._change_property_with_validity_check('_from_state', from_state)

    @property
    def from_outcome(self):
        """Property for the _from_outcome field

        """
        return self._from_outcome

    @from_outcome.setter
    @lock_state_machine
    @Observable.observed
    def from_outcome(self, from_outcome):
        if from_outcome is not None and not isinstance(from_outcome, int):
            raise ValueError("from_outcome must be of type int")

        self._change_property_with_validity_check('_from_outcome', from_outcome)

    @property
    def to_state(self):
        """Property for the _to_state field

        """
        return self._to_state

    @to_state.setter
    @lock_state_machine
    @Observable.observed
    def to_state(self, to_state):
        if to_state is not None and not isinstance(to_state, string_types):
            raise ValueError("to_state must be a string")

        self._change_property_with_validity_check('_to_state', to_state)

    @property
    def to_outcome(self):
        """Property for the to_outcome field

        """
        return self._to_outcome

    @to_outcome.setter
    @lock_state_machine
    @Observable.observed
    def to_outcome(self, to_outcome):
        if to_outcome is not None and not isinstance(to_outcome, int):
            raise ValueError("to_outcome must be of type int")

        self._change_property_with_validity_check('_to_outcome', to_outcome)

    @property
    def transition_id(self):
        """Property for the _transition_id field

        """
        return self._transition_id

