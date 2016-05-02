"""
.. module:: scoped_variable
   :platform: Unix, Windows
   :synopsis: A module for organizing the scoped variables in a container state

.. moduleauthor:: Sebastian Brunner


"""

import datetime
import time

from gtkmvc import Observable

from rafcon.statemachine.enums import DataPortType
from rafcon.statemachine.state_elements.data_port import DataPort
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

    :ivar str name: the name of the scoped variable
    :ivar data_type: specifies the type of the scoped variable (data port); the setter of _value will only allow
                assignments that satisfies the type constraint
    :ivar default_value: specifies the default value of the scoped variable (data port)
    :ivar int data_port_id: the id of the scoped variable (data port), must be unique for the parent state
    :ivar rafcon.statemachine.states.container_state.ContainerState parent: reference to the parent state

    """

    yaml_tag = u'!ScopedVariable'

    def __init__(self, name=None, data_type=None, default_value=None, scoped_variable_id=None, parent=None):

        Observable.__init__(self)

        DataPort.__init__(self, name, data_type, default_value, scoped_variable_id, parent)

    def __str__(self):
        return "ScopedVariable '{0}' [{1}] ({3} {2})".format(self.name, self.data_port_id, self.data_type,
                                                             self.default_value)

    @classmethod
    def from_dict(cls, dictionary):
        if 'scoped_variable_id' in dictionary:  # This is needed for backwards compatibility
            data_port_id = dictionary['scoped_variable_id']
        else:
            data_port_id = dictionary['data_port_id']
        name = dictionary['name']
        data_type = dictionary['data_type']
        default_value = dictionary['default_value']
        return cls(name, data_type, default_value, data_port_id)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'data_port_id': state_element.data_port_id,
            'name': state_element.name,
            'data_type': state_element.data_type,
            'default_value': state_element.default_value
        }


class ScopedData(Observable):
    """A class for representing scoped data of a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar name: the name of the scoped data
    :ivar from_state: the state_id of the state that wrote to the scoped data last
    :ivar value_type: specifies the type of self._value; the setter of __value will
            only allow assignments that satisfies the data_type constraint
    :ivar value: the current value of the scoped data
    :ivar data_port_type: the type of the data port that wrote to the scoped data last
    :ivar _timestamp: the timestamp when the scoped data was written to last

    """

    def __init__(self, name, value, value_type, from_state, data_port_type):

        Observable.__init__(self)

        self._from_state = None
        self._name = None
        self.from_state = from_state
        self.name = name

        self._value_type = type(None)
        if value_type is not None:
            self.value_type = value_type
        self._value = None
        self.value = value

        self._data_port_type = None
        self.data_port_type = data_port_type

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state)
        self._primary_key = None

    def __str__(self):
        return "ScopedData: \n name: %s \n data_type: %s \n value: %s \n from_state %s" % \
               (self.name, self.value_type, self.value, self.from_state)

    #########################################################################
    # Properties for all class field that must be observed by the gtkmvc
    #########################################################################

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, basestring):
            raise TypeError("key_name must be of type str")
        self._name = name
        # update key
        self._primary_key = self._name + self.from_state

    @property
    def value(self):
        """Property for the _value field

        """
        return self._value

    @value.setter
    @Observable.observed
    def value(self, value):
        # check for primitive data types
        if value is not None and not type_helpers.type_inherits_of_type(type(value), self.value_type):
            raise TypeError("Result must by of type '{0}'. Given: '{1}' with type '{2}'".format(
                self.value_type, value, type(value)
            ))
        # if value is not None and str(type(value).__name__) != self._value_type:
        #     print "types", value, str(type(value).__name__), self._value_type
        #     #check for classes
        #     if not isinstance(value, getattr(sys.modules[__name__], self._value_type)):
        #         raise TypeError("result must be of type %s" % str(self._value_type))
        self._timestamp = generate_time_stamp()
        # print "new scope data update {0}: {1} t:{2}".format(self.name+self.from_state, self._value, self._timestamp)
        self._value = value

    @property
    def value_type(self):
        """Property for the _value_type field

        """
        return self._value_type

    @value_type.setter
    @Observable.observed
    def value_type(self, value_type):
        self._value_type = type_helpers.convert_string_to_type(value_type)

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @Observable.observed
    def from_state(self, from_state):
        if not from_state is None:
            if not isinstance(from_state, basestring):
                raise TypeError("from_state must be of type str")
            if not self.name is None:  # this will just happen in __init__ when key_name is not yet initialized
                # update key
                self._primary_key = self.name + self._from_state
        self._from_state = from_state

    @property
    def data_port_type(self):
        """Property for the _data_port_type field

        """
        return self._data_port_type

    @data_port_type.setter
    @Observable.observed
    def data_port_type(self, data_port_type):
        if not isinstance(data_port_type, DataPortType):
            raise TypeError("data_port_type must be of type DataPortType")
        self._data_port_type = data_port_type

    @property
    def timestamp(self):
        """Property for the _timestamp field

        """
        return self._timestamp

    # WARNING: This setter function should never be used, as the timestamp is generated when the setter function of
    # the self._result variable is called
    @timestamp.setter
    @Observable.observed
    def timestamp(self, timestamp):
        if not isinstance(timestamp, float):
            raise TypeError("timestamp must be of type float")
        self._timestamp = timestamp
