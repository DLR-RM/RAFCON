"""
.. module:: scope_variable
   :platform: Unix, Windows
   :synopsis: A module for organizing the scoped variables in a container state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import sys
import time
import datetime

from statemachine.states.state import State


def generate_time_stamp():
    return time.time()


def get_human_readable_time(timestamp):
    return datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')


class ScopedVariable(Observable):

    """A class for representing a scoped variable in a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar _name: the key of the scoped variable
    :ivar _data_type: specifies the type of the scoped variable; the setter of _value will only allow assignments that
                satisfies the type constraint
    :ivar _from_state: the source state of the scoped variable
    :ivar _value_type: specifies the type of self._default_value
    :ivar _timestamp: the timestamp when the variable was written to last

    """

    def __init__(self, name=None, data_type=None, from_state=None, default_value=None):

        Observable.__init__(self)

        self._from_state = None
        self._name = None
        self.from_state = from_state
        self.name = name

        self._data_type = None
        self.data_type = data_type
        self._default_value = None
        self.default_value = default_value

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state.state_id
        self._primary_key = None

    def __str__(self):
        return "ScopedVariable: \n name: %s \n data_type: %s \n default_value: %s " %\
               (self.name, self.data_type, self.default_value)

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
            raise TypeError("name must be of type str")
        self._name = name
        #update key
        self._primary_key = self._name+self.from_state.state_id

    @property
    def default_value(self):
        """Property for the _default_value field

        """
        return self._default_value

    @default_value.setter
    @Observable.observed
    def default_value(self, default_value):
        if not default_value is None:
            #check for primitive data types
            if not str(type(default_value).__name__) == self._data_type:
                print self._data_type
                #check for classes
                if not isinstance(default_value, getattr(sys.modules[__name__], self._data_type)):
                    raise TypeError("default_value must be of type %s" % str(self._data_type))
        self._timestamp = generate_time_stamp()
        self._default_value = default_value

    @property
    def data_type(self):
        """Property for the _value_type field

        """
        return self._data_type

    @data_type.setter
    @Observable.observed
    def data_type(self, data_type):
        if not data_type is None:
            if not isinstance(data_type, str):
                raise TypeError("data_type must be of type str")
            if not data_type in ("int", "float", "bool", "str", "dict", "tuple", "list"):
                if not getattr(sys.modules[__name__], data_type):
                    raise TypeError("" + data_type + " is not a valid python data type")
        self._data_type = data_type

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @Observable.observed
    def from_state(self, from_state):
        if not from_state is None:
            if not isinstance(from_state, State):
                raise TypeError("from_state must be of type State")
            if not self.name is None:  #this will just happen in __init__ when key_name is not yet initialized
                #update key
                self._primary_key = self.name+self._from_state.state_id
        self._from_state = from_state

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


class ScopedData(Observable):

    """A class for representing scoped data of a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar _name: the name of the scoped data
    :ivar _value: the current value of the scoped data
    :ivar _value_type: specifies the type of self._value; the setter of __value will
            only allow assignments that satisfies the data_type constraint
    :ivar _from_state: the state that wrote to the scoped data last
    :ivar _timestamp: the timestamp when the scoped data was written to last

    """

    def __init__(self, name=None, value=None, value_type=None, from_state=None):

        Observable.__init__(self)

        self._from_state = None
        self._name = None
        self.from_state = from_state
        self.name = name

        self._value_type = None
        self.value_type = value_type
        self._value = None
        self.value = value

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state.state_id
        self._primary_key = None


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
        self._primary_key = self._name+self.from_state.state_id

    @property
    def value(self):
        """Property for the _value field

        """
        return self._value

    @value.setter
    @Observable.observed
    def value(self, value):
        #check for primitive data types
        if not str(type(value).__name__) == self._value_type:
            print self._value_type
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
            if not isinstance(from_state, State):
                raise TypeError("from_state must be of type State")
            if not self.name is None:  #this will just happen in __init__ when key_name is not yet initialized
                #update key
                self._primary_key = self.name+self._from_state.state_id
        self._from_state = from_state

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