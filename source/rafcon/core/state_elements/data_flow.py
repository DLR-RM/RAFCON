# Copyright (C) 2014-2018 DLR
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
.. module:: data_flow
   :synopsis: A module to represent a data flow in the state machine

"""

from weakref import ref
from future.utils import string_types
from gtkmvc3.observable import Observable

from rafcon.core.id_generator import generate_data_flow_id
from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config


class DataFlow(StateElement):
    """A class for representing a data flow connection in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar str DataFlow.from_state: the id of the source state of the data flow connection
    :ivar str DataFlow.to_state: the id of the target state of the data flow connection
    :ivar int DataFlow.from_key: the id of the data port / scoped variable of the source state
    :ivar int DataFlow.to_key: the id of the data port /scoped variable of the target state
    :ivar int DataFlow.data_flow_id: the id of the data port, must be unique for the parent state
    :ivar rafcon.core.states.container_state.ContainerState StateElement.parent: reference to the parent state
    """

    yaml_tag = u'!DataFlow'

    _data_flow_id = None
    _from_state = None
    _from_key = None
    _to_state = None
    _to_key = None

    def __init__(self, from_state=None, from_key=None, to_state=None, to_key=None, data_flow_id=None, parent=None,
                 safe_init=True):
        super(DataFlow, self).__init__(safe_init=safe_init)

        if data_flow_id is None:
            self._data_flow_id = generate_data_flow_id()
        else:
            if not isinstance(data_flow_id, int):
                raise ValueError("data_flow_id must be of type int")
            self._data_flow_id = data_flow_id

        if safe_init:
            DataFlow._safe_init(self, from_state, from_key, to_state, to_key, parent)
        else:
            DataFlow._unsafe_init(self, from_state, from_key, to_state, to_key, parent)

    def _safe_init(self, from_state, from_key, to_state, to_key, parent):
        self.from_state = from_state
        self.from_key = from_key
        self.to_state = to_state
        self.to_key = to_key
        # Checks for validity
        self.parent = parent

    def _unsafe_init(self, from_state, from_key, to_state, to_key, parent):
        self._from_state = from_state
        self._from_key = from_key
        self._to_state = to_state
        self._to_key = to_key
        if parent:
            self._parent = ref(parent)

    def __str__(self):
        default_string = "Data flow - from_state: %s, from_key: %s, to_state: %s, to_key: %s, id: %s" % \
               (self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id)
        if self.parent:
            from_port = None
            if self.from_state == self.parent.state_id:
                from_port = self.parent.get_data_port_by_id(self.from_key)
            else:
                from_port = self.parent.states[self.from_state].get_data_port_by_id(self.from_key)

            to_port = None
            if self.to_state == self.parent.state_id:
                to_port = self.parent.get_data_port_by_id(self.to_key)
            else:
                to_port = self.parent.states[self.to_state].get_data_port_by_id(self.to_key)

            if from_port and to_port:
                return "Data flow - from_state: %s, from_port_key: %s, from_port_name: %s, " \
                       "to_state: %s, to_port_key: %s, to_port_name: %s, data_flow_id: %s" % \
                       (self._from_state, self._from_key, from_port.name,
                        self._to_state, self._to_key, to_port.name, self._data_flow_id)
            else:
                return default_string
        else:
            return default_string

    def __copy__(self):
        return self.__class__(self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id,
                              None, safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        return self._data_flow_id

    @classmethod
    def from_dict(cls, dictionary):
        from_state = dictionary['from_state']
        from_key = dictionary['from_key']
        to_state = dictionary['to_state']
        to_key = dictionary['to_key']
        data_flow_id = dictionary['data_flow_id']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return cls(from_state, from_key, to_state, to_key, data_flow_id, safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'data_flow_id': state_element.data_flow_id,
            'from_state': state_element.from_state,
            'from_key': state_element.from_key,
            'to_state': state_element.to_state,
            'to_key': state_element.to_key
        }

#########################################################################
# Properties for all class field that must be observed by the gtkmvc3
#########################################################################

    @lock_state_machine
    @Observable.observed
    def modify_origin(self, from_state, from_key):
        """Set both from_state and from_key at the same time to modify data flow origin

        :param str from_state: State id of the origin state
        :param int from_key: Data port id of the origin port
        :raises exceptions.ValueError: If parameters have wrong types or the new data flow is not valid
        """
        if not isinstance(from_state, string_types):
            raise ValueError("Invalid data flow origin port: from_state must be a string")
        if not isinstance(from_key, int):
            raise ValueError("Invalid data flow origin port: from_key must be of type int")

        old_from_state = self.from_state
        old_from_key = self.from_key
        self._from_state = from_state
        self._from_key = from_key

        valid, message = self._check_validity()
        if not valid:
            self._from_state = old_from_state
            self._from_key = old_from_key
            raise ValueError("The data flow origin could not be changed: {0}".format(message))

    @property
    def from_state(self):
        """Property for the _from_state field
        """
        return self._from_state

    @from_state.setter
    @lock_state_machine
    # @Observable.observed  # should not be observed to stay consistent
    def from_state(self, from_state):
        if not isinstance(from_state, string_types):
            raise ValueError("from_state must be a string")

        self._change_property_with_validity_check('_from_state', from_state)

    @property
    def from_key(self):
        """Property for the _from_key field
        """
        return self._from_key

    @from_key.setter
    @lock_state_machine
    @Observable.observed
    def from_key(self, from_key):
        if not isinstance(from_key, int):
            raise ValueError("from_key must be of type int")

        self._change_property_with_validity_check('_from_key', from_key)

    @lock_state_machine
    @Observable.observed
    def modify_target(self, to_state, to_key):
        """Set both to_state and to_key at the same time to modify data flow target

        :param str to_state: State id of the target state
        :param int to_key: Data port id of the target port
        :raises exceptions.ValueError: If parameters have wrong types or the new data flow is not valid
        """
        if not isinstance(to_state, string_types):
            raise ValueError("Invalid data flow target port: from_state must be a string")
        if not isinstance(to_key, int):
            raise ValueError("Invalid data flow target port: from_outcome must be of type int")

        old_to_state = self.to_state
        old_to_key = self.to_key
        self._to_state = to_state
        self._to_key = to_key

        valid, message = self._check_validity()
        if not valid:
            self._to_state = old_to_state
            self._to_key = old_to_key
            raise ValueError("The data flow target could not be changed: {0}".format(message))

    @property
    def to_state(self):
        """Property for the _to_state field
        """
        return self._to_state

    @to_state.setter
    @lock_state_machine
    # @Observable.observed  # should not be observed to stay consistent
    def to_state(self, to_state):
        if not isinstance(to_state, string_types):
            raise ValueError("to_state must be a string")

        self._change_property_with_validity_check('_to_state', to_state)

    @property
    def to_key(self):
        """Property for the _to_key field
        """
        return self._to_key

    @to_key.setter
    @lock_state_machine
    @Observable.observed
    def to_key(self, to_key):
        if not isinstance(to_key, int):
            raise ValueError("to_key must be of type int")

        self._change_property_with_validity_check('_to_key', to_key)

    @property
    def data_flow_id(self):
        """Property for the _data_flow_id field

        """
        return self._data_flow_id
