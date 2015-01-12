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

    :ivar _name: the name of the scope variable
    :ivar _key: the key for the scope variable
    :ivar _from_state: the state that wrote to the scope variable last
    :ivar _value: the current value of the scope variable
    :ivar _value_type: specifies the type of the _value; the setter of _value will only allow assignments that
                satisfies the type constraint
    :ivar _timestamp: the timestamp when the variable was written to las

    """

    def __init__(self, key_name=None, value=None, value_type=None, from_state=None):

        Observable.__init__(self)

        self._value_type = None
        self.value_type = value_type
        self._value = None
        self.value = value

        self._from_state = None
        self._key_name = None
        self.from_state = from_state
        self.key_name = key_name

        self._timestamp = generate_time_stamp()
        # for storage purpose inside the container states (generated from key_name and from_state.state_id
        self._primary_key = None


#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @property
    def key_name(self):
        """Property for the _key_name field

        """
        return self._key_name

    @key_name.setter
    @Observable.observed
    def key_name(self, key_name):
        if not isinstance(key_name, str):
            raise TypeError("key_name must be of type str")
        self._key_name = key_name
        #update key
        self._primary_key = self._key_name+self.from_state.state_id

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
            if not self.key_name is None:  #this will just happen in __init__ when key_name is not yet initialized
                #update key
                self._primary_key = self.key_name+self._from_state.state_id
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


class ScopedResult(ScopedVariable):

    """A class for representing a scoped result in a container state

    It inherits from ScopeVariable as it needs the same fields.

    :ivar _key: the key of the result
    :ivar _from_state: the source state of the result
    :ivar _value: the current value of the result (newer results will overwrite the value)
    :ivar _value_type: specifies the type of the _result variable; the setter of _result will only allow assignments that
                satisfies the type constraint
    :ivar _timestamp: the data key of the target state

    """
    def __init__(self, key=None, value=None, value_type=None, from_state=None):

        ScopedVariable.__init__(self, key, value, value_type, from_state)