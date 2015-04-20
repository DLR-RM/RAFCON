"""
.. module:: data_port
   :platform: Unix, Windows
   :synopsis: A module to represent a data port of a state

.. moduleauthor:: Sebastian Brunner


"""

import sys
from gtkmvc import Observable
import yaml

from awesome_tool.statemachine.id_generator import generate_data_port_id
from awesome_tool.utils import log
logger = log.get_logger(__name__)


class DataPort(Observable, yaml.YAMLObject):

    #TODO: should the value of the data port be stored here as well?
    """A class for representing a data ports in a state

    :ivar name: the name of the data port
    :ivar data_type: the value type of the data port
    :ivar default_value: the default value of the data port

    """
    def __init__(self, name=None, data_type=None, default_value=None, data_port_id=None):

        Observable.__init__(self)

        if data_port_id is None:
            self._data_port_id = generate_data_port_id()
        else:
            self._data_port_id = data_port_id

        self._name = None
        self.name = name
        self._data_type = None
        self.data_type = data_type
        self._default_value = None
        self.default_value = default_value

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
            self.check_data_type(data_type)
            self._data_type = data_type
        except (TypeError, AttributeError) as e:
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
        try:
            self.check_data_type(data_type)
            default_value = self.check_default_value(default_value)

            self._data_type = data_type
            self._default_value = default_value
        except (TypeError, AttributeError) as e:
            logger.error("Could not change data type: {0}".format(e))

    @staticmethod
    def check_data_type(data_type):
        """Checks the data type

        Checks whether the passed data type is valid and throws an exception if not.
        :param data_type: The data type to check
        """
        if data_type is not None:
            if not isinstance(data_type, str):
                raise TypeError("data_type must be of type str")
            if data_type not in ("int", "float", "bool", "str", "dict", "tuple", "list"):
                try:
                    getattr(sys.modules[__name__], data_type)
                except AttributeError:
                    raise TypeError("" + data_type + " is not a valid python data type")

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
            # If the default value is passes as string, we have to convert it to the data type
            if isinstance(default_value, basestring):
                default_value = self.convert_string_to_type(default_value, data_type)
                if default_value is None:
                    raise AttributeError("Could not convert default value '{0}' to data type '{1}'".format(
                        default_value, data_type))

            # check for primitive data types
            if not str(type(default_value).__name__) == data_type:
                # check for classes
                if not isinstance(default_value, getattr(sys.modules[__name__], data_type)):
                    raise TypeError("Input of execute function must be of type %s" % str(data_type))
        return default_value

    @staticmethod
    def convert_string_to_type(string_value, data_type):
        """ Helper function to convert a given string to a given data type

        :param string_value: the string to convert
        :param data_type: the target data type
        :return: the converted value
        """
        import ast
        converted_value = None

        try:
            if data_type == "str":
                converted_value = str(string_value)
            elif data_type == "int":
                converted_value = int(string_value)
            elif data_type == "float":
                converted_value = float(string_value)
            elif data_type == "bool":
                converted_value = bool(string_value)
            elif data_type == "list":
                converted_value = ast.literal_eval(string_value)
            elif data_type == "dict":
                converted_value = ast.literal_eval(string_value)
            elif data_type == "tuple":
                converted_value = ast.literal_eval(string_value)
            else:
                logger.debug("No conversion from string to data type '{0}' defined".format(data_type))
        except (ValueError, SyntaxError):
            raise AttributeError("Can't convert '{0}' to type '{1}'".format(string_value, data_type))
        return converted_value