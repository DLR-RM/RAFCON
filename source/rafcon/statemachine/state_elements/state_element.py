"""
.. module:: state_element
   :platform: Unix, Windows
   :synopsis: A module to represent an abstract state element in the state machine

.. moduleauthor:: Franz Steinmetz


"""

from weakref import ref
from yaml import YAMLObject
from gtkmvc import Observable
from jsonconversion.jsonobject import JSONObject

from rafcon.statemachine.decorators import lock_state_machine
from rafcon.utils import log
from rafcon.utils.hashable import Hashable

logger = log.get_logger(__name__)


class StateElement(Observable, YAMLObject, JSONObject, Hashable):
    """A abstract base class for all elements of a state (ports, connections)

    It inherits from Observable to make a change of its fields observable. It also inherits from YAMLObject,
    in order to store/load it as YAML file.

    It raises an exceptions.NotImplementedError if the type of the class instance is type(self).

    :ivar rafcon.statemachine.states.state.State parent: Parent state of the state element
    """
    _parent = None

    yaml_tag = u'!StateElement'

    def __init__(self, parent=None):
        Observable.__init__(self)

        if type(self) == StateElement:
            raise NotImplementedError

        self.parent = parent

    def __eq__(self, other):
        # logger.info("compare method \n\t\t\t{0} \n\t\t\t{1}".format(self, other))
        if not isinstance(other, self.__class__):
            return False
        return str(self) == str(other)

    def __del__(self):
        self._parent = None

    @property
    def parent(self):
        """Getter for the parent state of the state element

        :return: None if parent is not defined, else the parent state
        :rtype: rafcon.statemachine.states.state.State
        """
        if not self._parent:
            return None
        return self._parent()

    @parent.setter
    @lock_state_machine
    @Observable.observed
    def parent(self, parent):
        """Setter for the parent state of the state element

        :param rafcon.statemachine.states.state.State parent: Parent state or None
        """
        if parent is None:
            self._parent = None
        else:
            from rafcon.statemachine.states.state import State
            assert isinstance(parent, State)

            old_parent = self.parent
            self._parent = ref(parent)

            valid, message = self._check_validity()
            if not valid:
                if not old_parent:
                    self._parent = None
                else:
                    self._parent = ref(old_parent)

                class_name = self.__class__.__name__
                raise ValueError("{0} invalid within state {1} (id {2}): {3}".format(class_name, parent.name,
                                                                                     parent.state_id, message))

    @property
    def state_element_id(self):
        """Returns the id of the state element
        """
        raise NotImplementedError()

    @property
    def core_element_id(self):
        """Returns the id of the state element
        """
        return self.state_element_id

    def to_dict(self):
        return self.state_element_to_dict(self)

    def update_hash(self, obj_hash):
        return Hashable.update_hash_from_dict(obj_hash, self.to_dict())

    @classmethod
    def from_dict(cls, dictionary):
        raise NotImplementedError()

    @staticmethod
    def state_element_to_dict(state_element):
        raise NotImplementedError()

    @classmethod
    def to_yaml(cls, dumper, state_element):
        dict_representation = cls.state_element_to_dict(state_element)
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        state_element = cls.from_dict(dict_representation)
        return state_element

    @lock_state_machine
    def _change_property_with_validity_check(self, property_name, value):
        """Helper method to change a property and reset it if the validity check fails

        :param str property_name: The name of the property to be changed, e.g. '_data_flow_id'
        :param value: The new desired value for this property
        :raises exceptions.ValueError: if a property could not be changed
        """
        assert isinstance(property_name, basestring)
        old_value = getattr(self, property_name)
        setattr(self, property_name, value)

        valid, message = self._check_validity()
        if not valid:
            setattr(self, property_name, old_value)
            class_name = self.__class__.__name__
            raise ValueError("The {2}'s '{0}' could not be changed: {1}".format(property_name[1:], message, class_name))

    def _check_validity(self):
        """Checks the validity of the state element's properties

        Some validity checks can only be performed by the parent. Thus, the existence of a parent and a check
        function must be ensured and this function be queried.

        :return: validity and messages
        :rtype: bool, str
        """
        from rafcon.statemachine.states.state import State

        if not self.parent:
            return True, "no parent"
        if not isinstance(self.parent, State):
            return True, "no parental check"
        return self.parent.check_child_validity(self)
