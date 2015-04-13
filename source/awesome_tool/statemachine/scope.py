"""
.. module:: scoped_variable
   :platform: Unix, Windows
   :synopsis: A module for organizing the scoped variables in a container state

.. moduleauthor:: Sebastian Brunner


"""

import sys
import time
import datetime

from gtkmvc import Observable
from awesome_tool.statemachine.enums import DataPortType
import yaml

from awesome_tool.statemachine.states.state import State
from awesome_tool.statemachine.data_port import DataPort


def generate_time_stamp():
    """
    Generate a time stamp for the current time.
    :return:
    """
    return time.time()


def get_human_readable_time(timestamp):
    """
    Converts a timestamp to a human readable format.
    :param timestamp: the timestamp to be converted
    :return: the converted timestamp
    """
    return datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')


class ScopedVariable(DataPort, Observable, yaml.YAMLObject):

    """A class for representing a scoped variable in a container state

    It inherits from the DataPort class as it needs exactly the same class fields.
    It inherits from Observable to make a change of its fields observable.

    :ivar _name: the key of the scoped variable
    :ivar _data_type: specifies the type of the scoped variable; the setter of _value will only allow assignments that
                satisfies the type constraint
    :ivar _value_type: specifies the type of self._default_value
    :ivar _timestamp: the timestamp when the variable was written to last

    """

    yaml_tag = u'!ScopedVariable'

    def __init__(self, name=None, data_type=None, default_value=None, scoped_variable_id=None):

        Observable.__init__(self)

        DataPort.__init__(self, name, data_type, default_value, scoped_variable_id)

    def __str__(self):
        return "ScopedVariable: \n name: %s \n data_type: %s \n default_value: %s " %\
               (self.name, self.data_type, self.default_value)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'scoped_variable_id': data.data_port_id,
            'name': data.name,
            'data_type': data.data_type,
            'default_value': data.default_value
        }
        node = dumper.represent_mapping(u'!ScopedVariable', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        scoped_variable_id = dict_representation['scoped_variable_id']
        name = dict_representation['name']
        data_type = dict_representation['data_type']
        default_value = dict_representation['default_value']
        return ScopedVariable(name, data_type, default_value, scoped_variable_id)


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

        self._value_type = None
        self.value_type = value_type
        self._value = None
        self.value = value

        self._data_port_type = None
        self.data_port_type = data_port_type

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state)
        self._primary_key = None

    def __str__(self):
        return "ScopedData: \n name: %s \n data_type: %s \n value: %s \n from_state %s" %\
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
        if not isinstance(name, str):
            raise TypeError("key_name must be of type str")
        self._name = name
        #update key
        self._primary_key = self._name+self.from_state

    @property
    def value(self):
        """Property for the _value field

        """
        return self._value

    @value.setter
    @Observable.observed
    def value(self, value):
        #check for primitive data types
        if value is not None and str(type(value).__name__) != self._value_type:
            print "types", value, str(type(value).__name__), self._value_type
            #check for classes
            if not isinstance(value, getattr(sys.modules[__name__], self._value_type)):
                raise TypeError("result must be of type %s" % str(self._value_type))
        self._timestamp = generate_time_stamp()
        self._value = value

    @property
    def value_type(self):
        """Property for the _value_type field

        """
        return self._value_type

    @value_type.setter
    @Observable.observed
    def value_type(self, value_type):
        if not isinstance(value_type, str):
            raise TypeError("result_type must be of type str")
        self._value_type = value_type

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @Observable.observed
    def from_state(self, from_state):
        if not from_state is None:
            if not isinstance(from_state, str):
                raise TypeError("from_state must be of type str")
            if not self.name is None:  #this will just happen in __init__ when key_name is not yet initialized
                #update key
                self._primary_key = self.name+self._from_state
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