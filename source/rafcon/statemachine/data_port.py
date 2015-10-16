"""
.. module:: data_port
   :platform: Unix, Windows
   :synopsis: A module to represent a data port of a state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import yaml

from rafcon.statemachine.id_generator import generate_data_port_id
from rafcon.utils import type_helpers
from rafcon.utils import log
logger = log.get_logger(__name__)


class DataPort(Observable, yaml.YAMLObject):
    """A class for representing a data ports in a state

    :ivar name: the name of the data port
    :ivar data_type: the value type of the data port
    :ivar default_value: the default value of the data port
    """

    # Define all parameters and set their default values
    _name = None
    _data_port_id = None
    _data_type = type(None)
    _default_value = None

    # Prevents validity checks by parent before all parameters are set
    _parent = None

    def __init__(self, name=None, data_type=None, default_value=None, data_port_id=None, parent=None, force_type=False):
        if type(self) == DataPort and not force_type:
            raise NotImplementedError
        Observable.__init__(self)

        if data_port_id is None:
            self._data_port_id = generate_data_port_id()
        else:
            self._data_port_id = data_port_id

        self.name = name

        if data_type is not None:
            self.data_type = data_type

        self.default_value = default_value

        # Checks for validity
        self.parent = parent

        logger.debug("DataPort with name %s initialized" % self.name)

    def __str__(self):
        return "DataPort '{0}' [{1}] ({3} {2})".format(self.name, self.data_port_id, self.data_type, self.default_value)

    yaml_tag = u'!DataPort'

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'data_port_id': data.data_port_id,
            'name': data.name,
            'data_type': data.data_type,
            'default_value': data.default_value
        }
        node = dumper.represent_mapping(cls.yaml_tag, dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        data_port_id = dict_representation['data_port_id']
        name = dict_representation['name']
        data_type = dict_representation['data_type']
        default_value = dict_representation['default_value']
        # Allow creation of DataPort class when loading from YAML file
        if cls == DataPort:
            return DataPort(name, data_type, default_value, data_port_id, force_type=True)
        # Call appropriate constructor, e.g. InputDataPort(...) for input data ports
        else:
            return cls(name, data_type, default_value, data_port_id, force_type=True)

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
    #########################################################################

    @property
    def data_port_id(self):
        """Property for the _data_port_id field

        """
        return self._data_port_id

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("Name must be of type str")

        if len(name) < 1:
            raise ValueError("Name cannot be empty")

        self.__change_property_with_validity_check('_name', name)

    @property
    def data_type(self):
        """Property for the _data_type field

        """
        return self._data_type

    @data_type.setter
    @Observable.observed
    def data_type(self, data_type):
        try:
            new_data_type = type_helpers.convert_string_to_type(data_type)
        except ValueError as e:
            raise ValueError("Could not change data type to '{0}': {1}".format(data_type, e))

        self.__change_property_with_validity_check('_data_type', new_data_type)

    @property
    def default_value(self):
        """Property for the _default_value field

        """
        return self._default_value

    @default_value.setter
    @Observable.observed
    def default_value(self, default_value):
        try:
            default_value = self.check_default_value(default_value)
            self._default_value = default_value
        except (TypeError, AttributeError) as e:
            raise e

    @property
    def parent(self):
        return self._parent

    @parent.setter
    @Observable.observed
    def parent(self, parent):
        if parent is not None:
            from rafcon.statemachine.states.state import State
            assert isinstance(parent, State)

        self.__change_property_with_validity_check('_parent', parent)

    @Observable.observed
    def change_data_type(self, data_type, default_value=None):
        """Changes the data type and the default value

        This method changes both the data type and default value. If one of the parameters does not fit,
        an exception is thrown and no property is changed. Using this method ensures a consistent data type
        and default value and only notifies once.
        :param data_type: The new data type
        :param default_value: The new default value
        :return:
        """
        old_data_type = self.data_type
        self.data_type = data_type

        if default_value is None:
            default_value = self.default_value

        if type_helpers.type_inherits_of_type(type(default_value), self._data_type):
            self._default_value = default_value
        else:

            if old_data_type.__name__ == "float" and data_type == "int":
                self._default_value = int(default_value)
            elif old_data_type.__name__ == "int" and data_type == "float":
                self._default_value = float(default_value)
            else:
                self._default_value = None

    def __change_property_with_validity_check(self, property_name, value):
        """Helper method to change a property and reset it if the validity check fails

        :param str property_name: The name of the property to be changed, e.g. '_data_port_id'
        :param value: The new desired value for this property
        """
        assert isinstance(property_name, str)
        old_value = getattr(self, property_name)
        setattr(self, property_name, value)

        valid, message = self._check_validity()
        if not valid:
            setattr(self, property_name, old_value)
            raise ValueError("The data port's '{0}' could not be changed: {1}".format(property_name[1:], message))

    def check_default_value(self, default_value, data_type=None):
        """Checks the default value

        Check whether the passed default value suits to to passed data type. If no data type is passed, the data type of
        the data port is used. If thh default value does not fit, an exception is thrown. If the default value is of
        type string, it is tried to convert that value to the data type.
        :param default_value: The default value to check
        :param data_type: The data type to use
        :return: The converted default value
        """
        if data_type is None:
            data_type = self.data_type

        if default_value is not None:
            # If the default value is passed as string, we have to convert it to the data type
            if isinstance(default_value, (str, basestring)):
                if len(default_value) > 1 and default_value[0] == '$':
                    return default_value
                if default_value == "None":
                    return None

                default_value = type_helpers.convert_string_value_to_type_value(default_value, data_type)
                if default_value is None:
                    raise AttributeError("Could not convert default value '{0}' to data type '{1}'".format(
                        default_value, data_type))

        return default_value

    def _check_validity(self):
        """Checks the validity of the data port properties

        Some validity checks can only be performed by the parent, e.g. data type changes when data flows are connected.
        Thus, the existence of a parent and a check function must be ensured and this function be queried.

        :return: (True, str message) if valid, (False, str reason) else
        """
        if not self.parent:
            return True, "no parent"
        if not hasattr(self.parent, 'check_child_validity') or \
                not callable(getattr(self.parent, 'check_child_validity')):
            return True, "no parental check"
        return self.parent.check_child_validity(self)


class InputDataPort(DataPort):

    yaml_tag = u'!InputDataPort'


class OutputDataPort(DataPort):

    yaml_tag = u'!OutputDataPort'
