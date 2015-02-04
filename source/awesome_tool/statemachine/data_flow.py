"""
.. module:: data_flow
   :platform: Unix, Windows
   :synopsis: A module to represent a data flow connection in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import yaml

from statemachine.states.state import State
from statemachine.id_generator import *


class DataFlow(Observable, yaml.YAMLObject):

    """A class for representing a data flow connection in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar _from_state: the source state of the data flow connection
    :ivar _from_key: the data key of the source state
    :ivar _to_state: the target state of the data flow connection
    :ivar _to_key: the data key of the target state

    """

    yaml_tag = u'!DataFlow'

    def __init__(self, from_state=None, from_key=None, to_state=None, to_key=None, data_flow_id=None):

        Observable.__init__(self)

        self._data_flow_id = None
        if data_flow_id is None:
            self.data_flow_id = generate_data_flow_id()
        else:
            self.data_flow_id = data_flow_id

        self._from_state = None
        self.from_state = from_state

        self._from_key = None
        self.from_key = from_key

        self._to_state = None
        self.to_state = to_state

        self._to_key = None
        self.to_key = to_key

    def __str__(self):
        return "Data flow - from_state: %s, from_key: %s, to_state: %s, to_key: %s, id: %s" % \
               (self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'from_state': data.from_state,
            'from_key': data.from_key,
            'to_state': data.to_state,
            'to_key': data.to_key
        }
        print dict_representation
        node = dumper.represent_mapping(u'!DataFlow', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        from_state = dict_representation['from_state']
        from_key = dict_representation['from_key']
        to_state = dict_representation['to_state']
        to_key = dict_representation['to_key']
        return DataFlow(from_state, from_key, to_state, to_key)

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
        if not isinstance(from_state, str):
            raise TypeError("from_state must be of type str")

        self._from_state = from_state

    @property
    def from_key(self):
        """Property for the _from_key field

        """
        return self._from_key

    @from_key.setter
    @Observable.observed
    def from_key(self, from_key):
        if not isinstance(from_key, int):
            raise TypeError("from_key must be of type int")

        self._from_key = from_key

    @property
    def to_state(self):
        """Property for the _to_state field

        """
        return self._to_state

    @to_state.setter
    @Observable.observed
    def to_state(self, to_state):
        if not isinstance(to_state, str):
            raise TypeError("to_state must be of type str")

        self._to_state = to_state

    @property
    def to_key(self):
        """Property for the _to_key field

        """
        return self._to_key

    @to_key.setter
    @Observable.observed
    def to_key(self, to_key):
        if not isinstance(to_key, int):
            raise TypeError("to_key must be of type int")

        self._to_key = to_key

    @property
    def data_flow_id(self):
        """Property for the _data_flow_id field

        """
        return self._data_flow_id

    @data_flow_id.setter
    @Observable.observed
    def data_flow_id(self, data_flow_id):
        if not data_flow_id is None:
            if not isinstance(data_flow_id, int):
                raise TypeError("data_flow_id must be of type int")

        self._data_flow_id = data_flow_id