# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Michael Vilzmann <michael.vilzmann@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: state_element
   :synopsis: A module to represent an abstract state element in the state machine

"""

from future.utils import string_types
from builtins import str
from weakref import ref
from yaml import YAMLObject
from gtkmvc3.observable import Observable
from jsonconversion.jsonobject import JSONObject

from rafcon.core.custom_exceptions import RecoveryModeException
from rafcon.core.config import global_config
from rafcon.core.decorators import lock_state_machine
from rafcon.utils import log
from rafcon.utils.hashable import Hashable

logger = log.get_logger(__name__)


class StateElement(Observable, YAMLObject, JSONObject, Hashable):
    """A abstract base class for all elements of a state (ports, connections)

    It inherits from Observable to make a change of its fields observable. It also inherits from YAMLObject,
    in order to store/load it as YAML file.

    It raises an exceptions.NotImplementedError if the type of the class instance is type(self).

    :ivar rafcon.core.states.state.State StateElement.parent: Parent state of the state element
    """
    _parent = None

    yaml_tag = u'!StateElement'

    def __init__(self, parent=None, safe_init=True):
        Observable.__init__(self)

        if type(self) == StateElement:
            raise NotImplementedError

        if safe_init:
            StateElement._safe_init(self, parent)
        else:
            StateElement._unsafe_init(self, parent)

    def __copy__(self):
        raise NotImplementedError("__copy__ method of class StateElement has to be implemented")

    def __deepcopy__(self, memo=None, _nil=[]):
        raise NotImplementedError("__deepcopy__ method of class StateElement has to be implemented")

    def _safe_init(self, parent):
        # uncomment this line when using LOAD_SM_WITH_CHECKS to check if unsafe_init triggers safe_init
        # raise Exception("Must not be executed during unsafe init")
        self.parent = parent

    def _unsafe_init(self, parent):
        if parent:
            self._parent = ref(parent)
        else:
            self._parent = None

    def __hash__(self):
        return id(self)

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        return str(self) == str(other)

    def __ne__(self, other):
        return not self.__eq__(other)

    def __cmp__(self, other):
        if isinstance(other, StateElement):
            if self.__class__ is other.__class__:
                if self.core_element_id < other.core_element_id:
                    return -1
                elif self.core_element_id > other.core_element_id:
                    return 1
                return 0
            return -1 if self.__class__.__name__ < other.__class__.__name__ else 1

    def __lt__(self, other):
        return self.__cmp__(other) < 0

    @property
    def parent(self):
        """Getter for the parent state of the state element

        :return: None if parent is not defined, else the parent state
        :rtype: rafcon.core.states.state.State
        """
        if not self._parent:
            return None
        return self._parent()

    @parent.setter
    @lock_state_machine
    @Observable.observed
    def parent(self, parent):
        """Setter for the parent state of the state element

        :param rafcon.core.states.state.State parent: Parent state or None
        """
        if parent is None:
            self._parent = None
        else:
            from rafcon.core.states.state import State
            assert isinstance(parent, State)

            old_parent = self.parent
            self._parent = ref(parent)

            valid, message = self._check_validity()
            if not valid:
                if not old_parent:
                    self._parent = None
                else:
                    self._parent = ref(old_parent)

                class_name = self.__class__.__name__
                if global_config.get_config_value("LIBRARY_RECOVERY_MODE") is True:
                    do_delete_item = True
                    # In case of just the data type is wrong raise an Exception but keep the data flow
                    if "not have matching data types" in message:
                        do_delete_item = False
                        self._parent = ref(parent)
                    raise RecoveryModeException("{0} invalid within state \"{1}\" (id {2}): {3}".format(
                        class_name, parent.name, parent.state_id, message), do_delete_item=do_delete_item)
                else:
                    raise ValueError("{0} invalid within state \"{1}\" (id {2}): {3} {4}".format(
                        class_name, parent.name, parent.state_id, message, self))

    @property
    def state_element_id(self):
        """Returns the id of the state element
        """
        raise NotImplementedError()

    @property
    def core_element_id(self):
        """Returns the id of the state element
        """
        return self.state_element_id

    def to_dict(self):
        return self.state_element_to_dict(self)

    def update_hash(self, obj_hash):
        return Hashable.update_hash_from_dict(obj_hash, self.to_dict())

    @classmethod
    def from_dict(cls, dictionary):
        raise NotImplementedError()

    @staticmethod
    def state_element_to_dict(state_element):
        raise NotImplementedError()

    @classmethod
    def to_yaml(cls, dumper, state_element):
        dict_representation = cls.state_element_to_dict(state_element)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state_element = cls.from_dict(dict_representation)
        return state_element

    @lock_state_machine
    def _change_property_with_validity_check(self, property_name, value):
        """Helper method to change a property and reset it if the validity check fails

        :param str property_name: The name of the property to be changed, e.g. '_data_flow_id'
        :param value: The new desired value for this property
        :raises exceptions.ValueError: if a property could not be changed
        """
        assert isinstance(property_name, string_types)
        old_value = getattr(self, property_name)
        setattr(self, property_name, value)

        valid, message = self._check_validity()
        if not valid:
            setattr(self, property_name, old_value)
            class_name = self.__class__.__name__
            raise ValueError("The {2}'s '{0}' could not be changed: {1}".format(property_name[1:], message, class_name))

    def _check_validity(self):
        """Checks the validity of the state element's properties

        Some validity checks can only be performed by the parent. Thus, the existence of a parent and a check
        function must be ensured and this function be queried.

        :return: validity and messages
        :rtype: bool, str
        """
        from rafcon.core.states.state import State

        if not self.parent:
            return True, "no parent"
        if not isinstance(self.parent, State):
            return True, "no parental check"
        return self.parent.check_child_validity(self)
