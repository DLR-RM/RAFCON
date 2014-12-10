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

    :ivar _from_state: the source state of the data flow connection
    :ivar _from_key: the data key of the source state
    :ivar _result: the target state of the data flow connection
    :ivar _type: specifies the type of the _result variable; the setter of _result will only allow assignments that
                satisfies the type constraint
    :ivar _timestamp: the data key of the target state

    """

    def __init__(self, from_state=None, from_key=None, result=None, type=None):

        Observable.__init__(self)

        self._from_state = from_state
        self._from_key = from_key
        self._result = result
        self._result_type = type
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
    def result(self):
        """Property for the _result field

        """
        return self._result

    @result.setter
    @Observable.observed
    def result(self, to_state):
        if isinstance(to_state, getattr(sys.modules[__name__], self._result_type)):
            raise TypeError("result must be of type %s" % self._result_type)
        self._timestamp = generate_time_stamp()
        self._result = to_state

    @property
    def result_type(self):
        """Property for the _result_type field

        """
        return self._result_type

    @result_type.setter
    @Observable.observed
    def result_type(self, result_type):
        if not isinstance(result_type, str):
            raise TypeError("result_type must be of type str")
        self._result_type = result_type

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