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
.. module:: outcome
   :synopsis: A module to represent an outcome in the state machine

"""
from weakref import ref
from future.utils import string_types
from gtkmvc3 import Observable

from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config

from rafcon.utils import log
logger = log.get_logger(__name__)


class LogicalPort(StateElement):
    """Base class for the logical ports"""

    def __init__(self, parent=None, safe_init=True):
        if self.__class__.__name__ == "LogicalPort":
            raise RuntimeError("The class LogicalPort is only a base class and must not be instantiated")
        super(LogicalPort, self).__init__(parent, safe_init=safe_init)


class Income(LogicalPort):
    """A class for representing an income of a state

    There can only be one Income within a state, which is why no income_id is required.

    :ivar rafcon.core.states.state.State StateElement.parent: reference to the parent state
    """

    yaml_tag = u'!Outcome'

    def __init__(self, parent=None, safe_init=True):
        super(Income, self).__init__(safe_init=safe_init)

        if safe_init:
            Income._safe_init(self, parent)
        else:
            Income._unsafe_init(self, parent)

    def _safe_init(self, parent):
        # Checks for validity
        self.parent = parent

    def _unsafe_init(self, parent):
        if parent:
            self._parent = ref(parent)

    def __str__(self):
        return "Income"

    def __copy__(self):
        return self.__class__(safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        """Always returns 0

        Only required for consistent handling of all state elements.

        :return: 0
        """
        return 0

    @classmethod
    def from_dict(cls, dictionary):
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return Income(safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {}

    @classmethod
    def from_yaml(cls, loader, node):
        return Income()


class Outcome(LogicalPort):
    """A class for representing an outcome of a state

    As the name of an outcome can be changes without modifying the transitions the primary key of an outcome is its
    id and not its name.

    :ivar int Outcome.outcome_id: the id of the outcome, must be unique on one hierarchy level
    :ivar str Outcome.name: the human readable name of the outcome
    :ivar rafcon.core.states.state.State StateElement.parent: reference to the parent state
    """

    yaml_tag = u'!Outcome'

    _outcome_id = None
    _name = None

    def __init__(self, outcome_id=None, name=None, parent=None, safe_init=True):
        """Constructor

        :param int outcome_id: State-wide unique Outcome ID
        :param str name: Name of the outcome
        :param rafcon.core.states.state.State parent: Reference to the parental state
        :raises TypeError: If `outcome_id` is not of type `int`
        """
        super(Outcome, self).__init__(safe_init=safe_init)

        if not isinstance(outcome_id, int):
            raise TypeError("outcome_id must be of type int")
        self._outcome_id = outcome_id

        if safe_init:
            Outcome._safe_init(self, name, parent)
        else:
            Outcome._unsafe_init(self, name, parent)

    def _safe_init(self, name, parent):
        self.name = name
        # Checks for validity
        self.parent = parent

    def _unsafe_init(self, name, parent):
        self._name = name
        if parent:
            self._parent = ref(parent)

    def __str__(self):
        return "Outcome '{0}' [{1}]".format(self.name, self.outcome_id)

    def __copy__(self):
        return self.__class__(self._outcome_id, self._name, None, safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        return self._outcome_id

    @classmethod
    def from_dict(cls, dictionary):
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return Outcome(dictionary['outcome_id'], dictionary['name'], safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'outcome_id': state_element.outcome_id,
            'name': state_element.name
        }

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        outcome_id = dict_representation['outcome_id']
        name = dict_representation['name']
        return Outcome(outcome_id, name)

# Properties for all class field that must be observed by the gtkmvc3
    @property
    def outcome_id(self):
        """ Returns the outcome_id """
        return self._outcome_id

    @property
    def name(self):
        """ Returns the Outcome's name """
        return self._name

    @name.setter
    @lock_state_machine
    @Observable.observed
    def name(self, name):
        if not isinstance(name, string_types):
            raise TypeError("name must be a string")

        if len(name) < 1:
            raise ValueError("Name cannot be empty")

        self._change_property_with_validity_check('_name', name)

