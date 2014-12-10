"""
.. module:: scope_variable
   :platform: Unix, Windows
   :synopsis: A module for representing a scoped variable in a container state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import numpy
import sys
import time
import datetime


def generate_time_stamp():
    return time.time()

def get_human_readable_time(timestamp):
    return datetime.datetime.fromtimestamp(timestamp).strftime('%Y-%m-%d %H:%M:%S')


class ScopeVariable(Observable):

    """A class for representing a scoped variable in a container state

    It inherits from Observable to make a change of its fields observable.

    :ivar _key_name: the key of the source state
    :ivar _from_state: the state that wrote to the scope variable last
    :ivar _value: the target state of the data flow connection
    :ivar _value_type: specifies the type of the _result variable; the setter of _result will only allow assignments that
                satisfies the type constraint
    :ivar _timestamp: the data key of the target state

    """

    def __init__(self, key_name=None, value_type=None, from_state=None, value=None):

        Observable.__init__(self)

        self._key_name = key_name
        self._value_type = type
        self._from_state = from_state
        self._value = value
        self._timestamp = generate_time_stamp()


#########################################################################
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @property
    def from_state(self):
        """Property for the _from_state field

        """
        return self._from_state

    @from_state.setter
    @Observable.observed
    def from_state(self, from_state):
        if not isinstance(from_state, numpy.uint32):
            raise TypeError("from_state must be of type numpy.uint32")

        self._from_state = from_state

    @property
    def from_key(self):
        """Property for the _from_key field

        """
        return self._from_key

    @from_key.setter
    @Observable.observed
    def from_key(self, from_key):
        if not isinstance(from_key, str):
            raise TypeError("from_key must be of type str")

        self._from_key = from_key

    @property
    def value(self):
        """Property for the _value field

        """
        return self._value

    @value.setter
    @Observable.observed
    def value(self, value):
        if isinstance(value, getattr(sys.modules[__name__], self._value_type)):
            raise TypeError("result must be of type %s" % self._value_type)
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