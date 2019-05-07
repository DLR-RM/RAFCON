# Copyright (C) 2014-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: vividict
   :synopsis: A module to represent hierarchical dictionaries which creates a new dictionary on the fly if a request
            uses a key that does not exist yet.

"""

from future.utils import string_types, native_str
from builtins import str
from yaml import YAMLObject
from jsonconversion.jsonobject import JSONObject


class Vividict(dict, YAMLObject, JSONObject):
    """Extended dictionary for arbitrary nested access

    A class which inherits from dict and can store an element for an arbitrary nested key. The single elements of the
    key do not have to exist beforehand.
    """

    #: Unique tag used for conversion to and from YAML objects
    yaml_tag = u'!Vividict'

    def __init__(self, dictionary=None):
        super(Vividict, self).__init__()
        if dictionary:
            self.set_dict(dictionary)

    def __missing__(self, key):
        """
        The main function of this class. If a key is missing it creates a new Vividict on the fly.
        :param key: the missing key for a value to be stored
        :return: the new value for the specified key
        """
        assert isinstance(key, string_types), "Vividict keys must be strings"
        key = str(key)
        value = self[key] = type(self)()
        return value

    def __setitem__(self, key, value):
        assert isinstance(key, string_types), "Vividict keys must be strings"
        key = str(key)
        if type(value) is dict:
            value = Vividict(value)
        super(Vividict, self).__setitem__(key, value)

    def set_dict(self, new_dict):
        """Sets the dictionary of the Vividict

        The method is able to handle nested dictionaries, by calling the method recursively.

        :param new_dict: The dict that will be added to the own dict
        """
        for key, value in new_dict.items():
            if isinstance(value, dict):
                self[str(key)] = Vividict(value)
            else:
                self[str(key)] = value

    def to_dict(self, *args, **kwargs):
        """Converts the Vividict to a common Python dict

        :return: A Python dict with all key-values pairs from this Vividict
        :rtype: dict
        """
        return self.vividict_to_dict(self, *args, **kwargs)

    @classmethod
    def from_dict(cls, dictionary):
        """Creates a Vividict from a (possibly nested) python dict

        :param dict dictionary: Python dict to be converted to a Vividict
        :return: A Vividict with all key-values pairs from the given dict
        :rtype: Vividict
        """
        return cls(dictionary)

    @staticmethod
    def vividict_to_dict(vividict, native_strings=False):
        """Helper method to create Python dicts from arbitrary Vividict objects

        :param Vividict vividict: A Vividict to be converted
        :return: A Python dict
        :rtype: dict
        """
        try:
            from numpy import ndarray
        except ImportError:
            ndarray = dict

        dictionary = {}

        def np_to_native(np_val):
            """Recursively convert numpy values to native Python values

            - Converts matrices to lists
            - Converts numpy.dtypes to float/int etc
            :param np_val: value to convert
            :return: value as native Python value
            """
            if isinstance(np_val, dict):
                for key, value in np_val.items():
                    np_val[key] = np_to_native(value)
            # The following condition cannot hold true if no numpy is installed, as ndarray is set to dict, which was
            # already handled in the previous condition
            elif isinstance(np_val, ndarray):
                # noinspection PyUnresolvedReferences
                np_val = np_val.tolist()
            if isinstance(np_val, (list, tuple)):
                native_list = [np_to_native(val) for val in np_val]
                if isinstance(np_val, tuple):
                    return tuple(native_list)
                return native_list
            if not hasattr(np_val, 'dtype'):  # Nothing to convert
                return np_val
            return np_val.item()  # Get the gloat/int etc value

        for key, value in vividict.items():
            if native_strings:  # e.g. converts newstr to str
                if isinstance(key, string_types):
                    key = native_str(key)
                if isinstance(value, string_types):
                    value = native_str(value)
            # Convert numpy values to native Python values
            value = np_to_native(value)

            # run recursively
            if isinstance(value, Vividict):
                value = Vividict.vividict_to_dict(value, native_strings)
            dictionary[key] = value

        return dictionary

    @classmethod
    def to_yaml(cls, dumper, vividict):
        """Implementation for the abstract method of the base class YAMLObject
        """
        dictionary = cls.vividict_to_dict(vividict, native_strings=True)
        node = dumper.represent_mapping(cls.yaml_tag, dictionary)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        """Implementation for the abstract method of the base class YAMLObject
        """
        dict_representation = loader.construct_mapping(node, deep=True)
        vividict = cls.from_dict(dict_representation)
        return vividict
