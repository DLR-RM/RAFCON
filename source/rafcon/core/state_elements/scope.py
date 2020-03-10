# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: scoped_variable
   :synopsis: A module for organizing the scoped variables in a container state

"""

from weakref import ref
from future.utils import string_types
import datetime
import time

from gtkmvc3.observable import Observable

from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.state_elements.data_port import DataPort
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config
from rafcon.utils import type_helpers


def generate_time_stamp():
    """
    Generate a time stamp for the current time as integer in micro.
    :return:
    """
    return int(round(time.time() * 1000000))


def get_human_readable_time(timestamp):
    """
    Converts a timestamp to a human readable format.
    :param timestamp: the timestamp to be converted
    :return: the converted timestamp
    """
    return datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')


class ScopedVariable(DataPort):
    """A class for representing a scoped variable in a container state

    It inherits from the DataPort class as it needs exactly the same class fields.
    It inherits from Observable to make a change of its fields observable.

    :ivar str DataPort.name: the name of the scoped variable
    :ivar type DataPort.data_type: specifies the type of the scoped variable (data port); the setter of _value will
                                   only allow assignments that satisfies the type constraint
    :ivar DataPort.default_value: specifies the default value of the scoped variable (data port)
    :ivar int scoped_variable_id: the id of the scoped variable (see DataPort.data_port_id),
                                  must be unique for the parent state
    :ivar rafcon.core.states.container_state.ContainerState StateElement.parent: reference to the parent state
    """

    yaml_tag = u'!ScopedVariable'

    def __init__(self, name=None, data_type=None, default_value=None, scoped_variable_id=None, parent=None,
                 safe_init=True):

        Observable.__init__(self)

        DataPort.__init__(self, name, data_type, default_value, scoped_variable_id, parent, safe_init=safe_init)

    def __str__(self):
        return "ScopedVariable '{0}' [{1}] ({3} {2})".format(self.name, self.data_port_id, self.data_type,
                                                             self.default_value)

    def __copy__(self):
        return self.__class__(self._name, self._data_type, self._default_value, self._data_port_id, None,
                              safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @classmethod
    def from_dict(cls, dictionary):
        if 'scoped_variable_id' in dictionary:  # This is needed for backwards compatibility
            data_port_id = dictionary['scoped_variable_id']
        else:
            data_port_id = dictionary['data_port_id']
        name = dictionary['name']
        data_type = dictionary['data_type']
        default_value = dictionary['default_value']
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return cls(name, data_type, default_value, data_port_id, safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'data_port_id': state_element.data_port_id,
            'name': state_element.name,
            'data_type': state_element.data_type,
            'default_value': state_element.default_value
        }


class ScopedData(StateElement):
    """A class for representing scoped data of a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar str ScopedData.name: the name of the scoped data
    :ivar str ScopedData.from_state: the state_id of the state that wrote to the scoped data last
    :ivar type value_type: specifies the type of self._value; the setter of __value will
                           only allow assignments that satisfies the data_type constraint
    :ivar value: the current value of the scoped data
    :ivar data_port_type: the type of the data port that wrote to the scoped data last
    :ivar str timestamp: the timestamp when the scoped data was written to last

    """
    _from_state = None
    _name = None
    _value_type = type(None)
    _value = None
    _data_port_type = None
    _primary_key = None

    def __init__(self, name, value, value_type, from_state, data_port_type, parent=None, safe_init=True):

        super(ScopedData, self).__init__(safe_init=safe_init)

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state)

        if safe_init:
            ScopedData._safe_init(self, name, value, value_type, from_state, data_port_type, parent)
        else:
            ScopedData._unsafe_init(self, name, value, value_type, from_state, data_port_type, parent)

        # logger.debug("DataPort with name %s initialized" % self.name)

    def _safe_init(self, name, value, value_type, from_state, data_port_type, parent):
        # from state is used for name self._primary_key
        self.from_state = from_state
        self.name = name
        self._primary_key = self.name + self._from_state
        if value_type is not None:
            self.value_type = value_type
        self.value = value
        self.data_port_type = data_port_type
        self.parent = parent

    def _unsafe_init(self, name, value, value_type, from_state, data_port_type, parent):
        self._from_state = from_state
        self._name = name
        if value_type is not None:
            self._value_type = value_type
        self._value = value
        self._data_port_type = data_port_type
        if parent:
            self._parent = ref(parent)

    def __copy__(self):
        return self.__class__(self._name, self._value, self._value_type, self._from_state, self._data_port_type,
                              parent=self.parent, safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        return self._primary_key

    def __str__(self):
        return "ScopedData: \n name: %s \n data_type: %s \n value: %s \n from_state %s" % \
               (self.name, self.value_type, self.value, self.from_state)

    @classmethod
    def from_dict(cls, dictionary):
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        return ScopedData(dictionary['name'],
                          dictionary['value'], dictionary['value_type'],
                          dictionary['from_state'], dictionary['data_port_type'], safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'name': state_element.name,
            'value': state_element.value,
            'value_type': state_element.value_type,
            'from_state': state_element.from_state,
            'data_port_type': state_element.data_port_type
        }

    #########################################################################
    # Properties for all class field that must be observed by the gtkmvc3
    #########################################################################

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @lock_state_machine
    @Observable.observed
    def name(self, name):
        if not isinstance(name, string_types):
            raise TypeError("key_name must be a string")
        self._name = name
        # update key
        self._primary_key = self._name + self.from_state

    @property
    def value(self):
        """Property for the _value field

        """
        return self._value

    @value.setter
    @lock_state_machine
    @Observable.observed
    def value(self, value):
        # check for primitive data types
        if value is not None and not type_helpers.type_inherits_of_type(type(value), self.value_type):
            raise TypeError("Result must by of type '{0}'. Given: '{1}' with type '{2}'".format(
                self.value_type, value, type(value)
            ))
        # if value is not None and str(type(value).__name__) != self._value_type:
        #     print("types", value, str(type(value).__name__), self._value_type)
        #     #check for classes
        #     if not isinstance(value, getattr(sys.modules[__name__], self._value_type)):
        #         raise TypeError("result must be of type %s" % str(self._value_type))
        self._timestamp = generate_time_stamp()
        # print("new scope data update {0}: {1} t:{2}".format(self.name+self.from_state, self._value, self._timestamp))
        self._value = value

    @property
    def value_type(self):
        """Property for the _value_type field

        """
        return self._value_type

    @value_type.setter
    @lock_state_machine
    @Observable.observed
    def value_type(self, value_type):
        self._value_type = type_helpers.convert_string_to_type(value_type)

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @lock_state_machine
    @Observable.observed
    def from_state(self, from_state):
        if from_state is not None:
            if not isinstance(from_state, string_types):
                raise TypeError("from_state must be a string")
            if self.name is not None:  # this will just happen in __init__ when key_name is not yet initialized
                # update key
                self._primary_key = self.name + self._from_state
        self._from_state = from_state

    @property
    def data_port_type(self):
        """Property for the _data_port_type field

        """
        return self._data_port_type

    @data_port_type.setter
    @lock_state_machine
    @Observable.observed
    def data_port_type(self, data_port_type):
        if not issubclass(data_port_type, DataPort):
            raise TypeError("data_port_type must be a subclass of DataPort")
        self._data_port_type = data_port_type

    @property
    def timestamp(self):
        """Property for the _timestamp field

        """
        return self._timestamp

    # WARNING: This setter function should never be used, as the timestamp is generated when the setter function of
    # the self._result variable is called
    @timestamp.setter
    @lock_state_machine
    @Observable.observed
    def timestamp(self, timestamp):
        if not isinstance(timestamp, float):
            raise TypeError("timestamp must be of type float")
        self._timestamp = timestamp
