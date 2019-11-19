# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: data_port
   :synopsis: A module to represent data ports the state machine

"""
from weakref import ref
from future.utils import string_types
from enum import Enum
from gtkmvc3.observable import Observable

from rafcon.core.id_generator import generate_data_port_id
from rafcon.core.state_elements.state_element import StateElement
from rafcon.core.decorators import lock_state_machine
from rafcon.core.config import global_config
from rafcon.utils import log
from rafcon.utils import type_helpers
logger = log.get_logger(__name__)


class DataPort(StateElement):
    """A class for representing a data ports in a state

    :ivar str DataPort.name: the name of the data port
    :ivar type DataPort.data_type: the value type of the data port can be handed as convertible :class:`str` too
    :ivar DataPort.default_value: the default value of the data port
    :ivar int data_port_id: the id of the data port, must be unique for the parent state
    :ivar rafcon.core.states.state.State StateElement.parent: reference to the parent state
    :ivar bool DataPort.force_type: if true the DataPort type exception is not raised while initiation
                                    (backward compatibility)
    :ivar bool DataPort.init_without_default_value_type_exceptions: if true it is allowed to initiate with any default
                                                                    value type used to load not matching default value
                                                                    data types and correct them using the GUI.
    """

    # Define all parameters and set their default values
    _name = None
    _data_port_id = None
    _data_type = type(None)
    _default_value = None

    def __init__(self, name=None, data_type=None, default_value=None, data_port_id=None, parent=None, force_type=False,
                 init_without_default_value_type_exceptions=False, safe_init=True):
        if type(self) == DataPort and not force_type:
            raise NotImplementedError
        super(DataPort, self).__init__(safe_init=safe_init)
        self._no_type_error_exceptions = True if init_without_default_value_type_exceptions else False
        self._was_forced_type = force_type
        if data_port_id is None:
            self._data_port_id = generate_data_port_id([])
            logger.warning("Look out: Instantiation of a data port without specifying its id is not recommended! The "
                        "add_data_port* functions of the State/ContainerState class should be used!")
        else:
            self._data_port_id = data_port_id

        self._no_type_error_exceptions = False

        if safe_init:
            DataPort._safe_init(self, name, data_type, default_value, parent)
        else:
            DataPort._unsafe_init(self, name, data_type, default_value, parent)

        # logger.debug("DataPort with name %s initialized" % self.name)

    def _safe_init(self, name, data_type, default_value, parent):
        self.name = name
        if data_type is not None:
            self.data_type = data_type
        self.default_value = default_value
        # Checks for validity
        self.parent = parent

    def _unsafe_init(self, name, data_type, default_value, parent):
        self._name = name
        if data_type is not None:
            self._data_type = data_type
        if isinstance(default_value, string_types):
            self._default_value = type_helpers.convert_string_value_to_type_value(default_value, data_type)
        else:
            self._default_value = default_value
        if parent:
            self._parent = ref(parent)

    def __str__(self):
        return "DataPort '{0}' [{1}] ({3} {2})".format(self.name, self.data_port_id, self.data_type, self.default_value)

    yaml_tag = u'!DataPort'

    def __copy__(self):
        return self.__class__(self._name, self._data_type, self._default_value, self._data_port_id, None,
                              self._was_forced_type, safe_init=False)

    def __deepcopy__(self, memo=None, _nil=[]):
        return self.__copy__()

    @property
    def state_element_id(self):
        return self._data_port_id

    @classmethod
    def from_dict(cls, dictionary):
        data_port_id = dictionary['data_port_id']
        name = dictionary['name']
        data_type = dictionary['data_type']
        default_value = dictionary['default_value']
        # Allow creation of DataPort class when loading from YAML file
        safe_init = global_config.get_config_value("LOAD_SM_WITH_CHECKS", True)
        if cls == DataPort:
            return DataPort(name, data_type, default_value, data_port_id, force_type=True,
                            init_without_default_value_type_exceptions=True, safe_init=safe_init)
        # Call appropriate constructor, e.g. InputDataPort(...) for input data ports
        else:
            return cls(name, data_type, default_value, data_port_id, force_type=True,
                       init_without_default_value_type_exceptions=True, safe_init=safe_init)

    @staticmethod
    def state_element_to_dict(state_element):
        return {
            'data_port_id': state_element.data_port_id,
            'name': state_element.name,
            'data_type': state_element.data_type,
            'default_value': state_element.default_value
        }

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
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
    @lock_state_machine
    @Observable.observed
    def name(self, name):
        if not isinstance(name, string_types):
            raise TypeError("Name must be a string")

        if len(name) < 1:
            raise ValueError("Name cannot be empty")

        if name == "error":
            logger.warning("The name of the created data port is 'error'. "
                        "This name is internally used for error propagation as well. "
                        "Only proceed if you know, what you are doing, otherwise rename the data port.")
        self._change_property_with_validity_check('_name', name)

    @property
    def data_type(self):
        """Property for the _data_type field

        """
        return self._data_type

    @data_type.setter
    @lock_state_machine
    @Observable.observed
    def data_type(self, data_type):
        try:
            new_data_type = type_helpers.convert_string_to_type(data_type)
        except ValueError as e:
            raise ValueError("Could not change data type to '{0}': {1}".format(data_type, e))

        self._change_property_with_validity_check('_data_type', new_data_type)

    @property
    def default_value(self):
        """Property for the _default_value field

        """
        return self._default_value

    @default_value.setter
    @lock_state_machine
    @Observable.observed
    def default_value(self, default_value):
        try:
            default_value = self.check_default_value(default_value)
            self._default_value = default_value
        except (TypeError, AttributeError) as e:
            raise e

    @lock_state_machine
    @Observable.observed
    def change_data_type(self, data_type, default_value=None):
        """This method changes both the data type and default value. If one of the parameters does not fit,
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
                if self.default_value:
                    self._default_value = int(default_value)
                else:
                    self._default_value = 0
            elif old_data_type.__name__ == "int" and data_type == "float":
                if self.default_value:
                    self._default_value = float(default_value)
                else:
                    self._default_value = 0.0
            else:
                self._default_value = None

    def check_default_value(self, default_value, data_type=None):
        """Check whether the passed default value suits to the passed data type. If no data type is passed, the
        data type of the data port is used. If the default value does not fit, an exception is thrown. If the default
        value is of type string, it is tried to convert that value to the data type.

        :param default_value: The default value to check
        :param data_type: The data type to use
        :raises exceptions.AttributeError: if check fails
        :return: The converted default value
        """
        if data_type is None:
            data_type = self.data_type

        if default_value is not None:
            # If the default value is passed as string, we have to convert it to the data type
            if isinstance(default_value, string_types):
                if len(default_value) > 1 and default_value[0] == '$':
                    return default_value
                if default_value == "None":
                    return None

                default_value = type_helpers.convert_string_value_to_type_value(default_value, data_type)
                if default_value is None:
                    raise AttributeError("Could not convert default value '{0}' to data type '{1}'.".format(
                        default_value, data_type))
            else:
                if not isinstance(default_value, self.data_type):
                    if self._no_type_error_exceptions:
                        logger.warning("Handed default value '{0}' is of type '{1}' but data port data type is {2} {3}."
                                       "".format(default_value, type(default_value), data_type, self))
                    else:
                        raise TypeError("Handed default value '{0}' is of type '{1}' but data port data type is {2}"
                                        "{3} of {4}.".format(default_value, type(default_value), data_type,
                                                             self,
                                                             self.parent.get_path() if self.parent is not None else ""))

        return default_value


class InputDataPort(DataPort):

    yaml_tag = u'!InputDataPort'


class OutputDataPort(DataPort):

    yaml_tag = u'!OutputDataPort'
