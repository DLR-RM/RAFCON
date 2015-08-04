"""
.. module:: data_port
   :platform: Unix, Windows
   :synopsis: A module to represent a data port of a state

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import yaml

from awesome_tool.statemachine.id_generator import generate_data_port_id
from awesome_tool.utils import type_helpers
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class DataPort(Observable, yaml.YAMLObject):

    #TODO: should the value of the data port be stored here as well?
    """A class for representing a data ports in a state

    :ivar name: the name of the data port
    :ivar data_type: the value type of the data port
    :ivar default_value: the default value of the data port

    """
    def __init__(self, name=None, data_type=None, default_value=None, data_port_id=None, parent=None):

        Observable.__init__(self)

        if data_port_id is None:
            self._data_port_id = generate_data_port_id()
        else:
            self._data_port_id = data_port_id

        self._name = None
        self.name = name
        self._data_type = type(None)
        if data_type is not None:
            self.data_type = data_type
        self._default_value = None
        self.default_value = default_value

        self._parent = None
        self.parent = parent

        logger.debug("DataPort with name %s initialized" % self.name)

    def __str__(self):
        return "DataPort: \n name: %s \n data_type: %s \n default_value: %s " % (self.name, self.data_type,
                                                                                 self.default_value)

    yaml_tag = u'!DataPort'

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_representation = {
            'data_port_id': data.data_port_id,
            'name': data.name,
            'data_type': data.data_type,
            'default_value': data.default_value
        }
        node = dumper.represent_mapping(u'!DataPort', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node)
        data_port_id = dict_representation['data_port_id']
        name = dict_representation['name']
        data_type = dict_representation['data_type']
        default_value = dict_representation['default_value']
        return DataPort(name, data_type, default_value, data_port_id)

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
        self._name = name

    @property
    def data_type(self):
        """Property for the _data_type field

        """
        return self._data_type

    @data_type.setter
    @Observable.observed
    def data_type(self, data_type):
        try:
            # self.check_data_type(data_type)
            self._data_type = type_helpers.convert_string_to_type(data_type)
        except ValueError as e:
            raise e

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
            from awesome_tool.statemachine.states.state import State
            assert isinstance(parent, State)
        self._parent = parent

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
        if default_value is None:
            default_value = self.default_value
        try:
            self._data_type = type_helpers.convert_string_to_type(data_type)

            if type_helpers.type_inherits_of_type(type(default_value), self._data_type):
                self._default_value = default_value
            else:
                self._default_value = None
        except (TypeError, AttributeError, ValueError) as e:
            logger.error("Could not change data type: {0}".format(e))

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
                if len(default_value) == 0 or default_value == "None":
                    return None

                default_value = type_helpers.convert_string_value_to_type_value(default_value, data_type)
                if default_value is None:
                    raise AttributeError("Could not convert default value '{0}' to data type '{1}'".format(
                        default_value, data_type))

        return default_value
