"""
.. module:: data_flow
   :platform: Unix, Windows
   :synopsis: A module to represent a data flow connection in the statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import yaml

from awesome_tool.statemachine.id_generator import *


class DataFlow(Observable, yaml.YAMLObject):

    """A class for representing a data flow connection in the state machine

    It inherits from Observable to make a change of its fields observable.

    :ivar from_state: the id of the source state of the data flow connection
    :ivar to_state: the id of the target state of the data flow connection
    :ivar from_key: the id of the data port / scoped variable of the source state
    :ivar to_key: the id of the data port /scoped variable of the target state

    """

    yaml_tag = u'!DataFlow'

    def __init__(self, from_state=None, from_key=None, to_state=None, to_key=None, data_flow_id=None, parent=None):
        Observable.__init__(self)

        # Prevents validity checks by parent before all parameters are set
        self._parent = None

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

        self.parent = parent
        if not self._check_validity():
            raise ValueError("Could not create transition. The parameters for the new transition are not valid.")

    def __str__(self):
        return "Data flow - from_state: %s, from_key: %s, to_state: %s, to_key: %s, id: %s" % \
               (self._from_state, self._from_key, self._to_state, self._to_key, self._data_flow_id)

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'data_flow_id': data.data_flow_id,
            'from_state': data.from_state,
            'from_key': data.from_key,
            'to_state': data.to_state,
            'to_key': data.to_key
        }
        node = dumper.represent_mapping(u'!DataFlow', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        from_state = dict_representation['from_state']
        from_key = dict_representation['from_key']
        to_state = dict_representation['to_state']
        to_key = dict_representation['to_key']
        data_flow_id = dict_representation['data_flow_id']
        return DataFlow(from_state, from_key, to_state, to_key, data_flow_id)

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
        if not isinstance(from_state, str):
            raise TypeError("from_state must be of type str")
        if not isinstance(from_key, int):
            raise TypeError("from_key must be of type int")

        old_from_state = self.from_state
        old_from_key = self.from_key
        self._from_state = from_state
        self._from_key = from_key

        if not self._check_validity():
            self._from_state = old_from_state
            self._from_key = old_from_key
            raise ValueError("The parent state refused to change the origin of the data flow")

    @property
    def from_state(self):
        """Property for the _from_state field
        """
        return self._from_state

    @from_state.setter
    # @Observable.observed  # should not be observed to stay consistent
    def from_state(self, from_state):
        if not isinstance(from_state, str):
            raise TypeError("from_state must be of type str")

        self._from_state = from_state
        self.__change_property_with_validity_check('_from_state', from_state)

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

        self.__change_property_with_validity_check('_from_key', from_key)

    @Observable.observed
    def modify_target(self, to_state, to_key):
        """ Set to_state and to_key (Data Port) at ones to support fully valid transition modifications.
        :param str to_state: valid target state
        :param int to_key: valid target data port
        :return:
        """
        if not isinstance(to_state, str):
            raise TypeError("from_state must be of type str")
        if not isinstance(to_key, int):
            raise TypeError("from_outcome must be of type int")

        old_to_state = self.to_state
        old_to_key = self.to_key
        self._to_state = to_state
        self._to_key = to_key

        if not self._check_validity():
            self._to_state = old_to_state
            self._to_key = old_to_key
            raise ValueError("The parent state refused to change the target of the data flow")

    @property
    def to_state(self):
        """Property for the _to_state field
        """
        return self._to_state

    @to_state.setter
    # @Observable.observed  # should not be observed to stay consistent
    def to_state(self, to_state):
        if not isinstance(to_state, str):
            raise TypeError("to_state must be of type str")

        self.__change_property_with_validity_check('_to_state', to_state)

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

        self.__change_property_with_validity_check('_to_key', to_key)

    @property
    def data_flow_id(self):
        """Property for the _data_flow_id field

        """
        return self._data_flow_id

    @data_flow_id.setter
    @Observable.observed
    def data_flow_id(self, data_flow_id):
        if data_flow_id is not None:
            if not isinstance(data_flow_id, int):
                raise TypeError("data_flow_id must be of type int")

        self.__change_property_with_validity_check('_data_flow_id', data_flow_id)

    @property
    def parent(self):
        return self._parent

    @parent.setter
    @Observable.observed
    def parent(self, parent):
        if parent is not None:
            from awesome_tool.statemachine.states.container_state import ContainerState
            assert isinstance(parent, ContainerState)
        self._parent = parent

    def __change_property_with_validity_check(self, property_name, value):
        """Helper method to change a property and reset it if the validity check fails

        :param str property_name: The name of the property to be changed, e.g. '_data_flow_id'
        :param value: The new desired value for this property
        """
        assert isinstance(property_name, str)
        old_value = getattr(self, property_name)
        setattr(self, property_name, value)

        if not self._check_validity():
            setattr(self, property_name, old_value)
            raise ValueError("The parent state refused to change the '{0}' of the data flow".format(property_name[1:]))

    def _check_validity(self):
        """Checks the validity of the data flow properties

        Some validity checks can only be performed by the parent, e.g. checks for already connected data ports.
        Thus, the existence of a parent and a check function must be ensured and this function be queried.

        :return: True if valid, False else
        """
        if not self.parent:
            return True
        if not hasattr(self.parent, 'check_child_validity') or \
                not callable(getattr(self.parent, 'check_child_validity')):
            return True
        if self.parent.check_child_validity(self):
            return True