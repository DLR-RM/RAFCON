"""
.. module:: data_flow
   :platform: Unix, Windows
   :synopsis: A module to represent a data flow in the state machine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable

from rafcon.statemachine.id_generator import generate_data_flow_id
from rafcon.statemachine.state_elements.state_element import StateElement


class DataFlow(StateElement):
    """A class for representing a data flow connection in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar str from_state: the id of the source state of the data flow connection
    :ivar str to_state: the id of the target state of the data flow connection
    :ivar int from_key: the id of the data port / scoped variable of the source state
    :ivar int to_key: the id of the data port /scoped variable of the target state
    :ivar int data_flow_id: the id of the data port, must be unique for the parent state
    :ivar rafcon.statemachine.states.container_state.ContainerState parent: reference to the parent state
    """

    yaml_tag = u'!DataFlow'

    _data_flow_id = None
    _from_state = None
    _from_key = None
    _to_state = None
    _to_key = None

    def __init__(self, from_state=None, from_key=None, to_state=None, to_key=None, data_flow_id=None, parent=None):
        super(DataFlow, self).__init__()

        if data_flow_id is None:
            self._data_flow_id = generate_data_flow_id()
        else:
            if not isinstance(data_flow_id, int):
                raise ValueError("data_flow_id must be of type int")
            self._data_flow_id = data_flow_id

        self.from_state = from_state
        self.from_key = from_key
        self.to_state = to_state
        self.to_key = to_key

        # Checks for validity
        self.parent = parent

    def __str__(self):
        return "Data flow - from_state: %s, from_key: %s, to_state: %s, to_key: %s, id: %s" % \
               (self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id)

    def __copy__(self):
        return self.__class__(self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id, None)

    __deepcopy__ = __copy__

    @classmethod
    def from_dict(cls, dictionary):
        from_state = dictionary['from_state']
        from_key = dictionary['from_key']
        to_state = dictionary['to_state']
        to_key = dictionary['to_key']
        data_flow_id = dictionary['data_flow_id']
        return cls(from_state, from_key, to_state, to_key, data_flow_id)

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
# Properties for all class field that must be observed by the gtkmvc
#########################################################################

    @Observable.observed
    def modify_origin(self, from_state, from_key):
        """ Set from_state and from_outcome at ones to support fully valid transition modifications.
        :param str from_state: valid origin state
        :param int from_key: valid origin outcome
        :return:
        """
        if not isinstance(from_state, basestring):
            raise ValueError("from_state must be of type str")
        if not isinstance(from_key, int):
            raise ValueError("from_key must be of type int")

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
    # @Observable.observed  # should not be observed to stay consistent
    def from_state(self, from_state):
        if not isinstance(from_state, basestring):
            raise ValueError("from_state must be of type str")

        self._change_property_with_validity_check('_from_state', from_state)

    @property
    def from_key(self):
        """Property for the _from_key field
        """
        return self._from_key

    @from_key.setter
    @Observable.observed
    def from_key(self, from_key):
        if not isinstance(from_key, int):
            raise ValueError("from_key must be of type int")

        self._change_property_with_validity_check('_from_key', from_key)

    @Observable.observed
    def modify_target(self, to_state, to_key):
        """ Set to_state and to_key (Data Port) at ones to support fully valid transition modifications.
        :param str to_state: valid target state
        :param int to_key: valid target data port
        :return:
        """
        if not isinstance(to_state, basestring):
            raise ValueError("from_state must be of type str")
        if not isinstance(to_key, int):
            raise ValueError("from_outcome must be of type int")

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
    # @Observable.observed  # should not be observed to stay consistent
    def to_state(self, to_state):
        if not isinstance(to_state, basestring):
            raise ValueError("to_state must be of type str")

        self._change_property_with_validity_check('_to_state', to_state)

    @property
    def to_key(self):
        """Property for the _to_key field
        """
        return self._to_key

    @to_key.setter
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
