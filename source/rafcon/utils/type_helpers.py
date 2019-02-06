# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: type_helpers
   :synopsis: A module holding all utility functions for converting types

"""



from future import standard_library
standard_library.install_aliases()
from past.builtins import str
from builtins import str
from pydoc import locate, ErrorDuringImport
from inspect import isclass
import sys


def convert_string_to_type(string_value):
    """Converts a string into a type or class

    :param string_value: the string to be converted, e.g. "int"
    :return: The type derived from string_value, e.g. int
    """
    # If the parameter is already a type, return it
    if string_value in ['None', type(None).__name__]:
        return type(None)
    if isinstance(string_value, type) or isclass(string_value):
        return string_value

    # Get object associated with string
    # First check whether we are having a built in type (int, str, etc)
    if sys.version_info >= (3,):
        import builtins as builtins23
    else:
        import __builtin__ as builtins23
    if hasattr(builtins23, string_value):
        obj = getattr(builtins23, string_value)
        if type(obj) is type:
            return obj
    # If not, try to locate the module
    try:
        obj = locate(string_value)
    except ErrorDuringImport as e:
        raise ValueError("Unknown type '{0}'".format(e))

    # Check whether object is a type
    if type(obj) is type:
        return locate(string_value)

    # Check whether object is a class
    if isclass(obj):
        return obj

    # Raise error if none is the case
    raise ValueError("Unknown type '{0}'".format(string_value))


def convert_string_value_to_type_value(string_value, data_type):
    """Helper function to convert a given string to a given data type

    :param str string_value: the string to convert
    :param type data_type: the target data type
    :return: the converted value
    """
    from ast import literal_eval

    try:
        if data_type in (str, type(None)):
            converted_value = str(string_value)
        elif data_type == int:
            converted_value = int(string_value)
        elif data_type == float:
            converted_value = float(string_value)
        elif data_type == bool:
            converted_value = bool(literal_eval(string_value))
        elif data_type in (list, dict, tuple):
            converted_value = literal_eval(string_value)
            if type(converted_value) != data_type:
                raise ValueError("Invalid syntax: {0}".format(string_value))
        elif data_type == object:
            try:
                converted_value = literal_eval(string_value)
            except (ValueError, SyntaxError):
                converted_value = literal_eval('"' + string_value + '"')
        elif isinstance(data_type, type):  # Try native type conversion
            converted_value = data_type(string_value)
        elif isclass(data_type):  # Call class constructor
            converted_value = data_type(string_value)
        else:
            raise ValueError("No conversion from string '{0}' to data type '{0}' defined".format(
                string_value, data_type.__name__))
    except (ValueError, SyntaxError, TypeError) as e:
        raise AttributeError("Can't convert '{0}' to type '{1}': {2}".format(string_value, data_type.__name__, e))
    return converted_value


def type_inherits_of_type(inheriting_type, base_type):
    """Checks whether inheriting_type inherits from base_type

    :param str inheriting_type:
    :param str base_type:
    :return: True is base_type is base of inheriting_type
    """
    assert isinstance(inheriting_type, type) or isclass(inheriting_type)
    assert isinstance(base_type, type) or isclass(base_type)

    if inheriting_type == base_type:
        return True
    else:
        if len(inheriting_type.__bases__) != 1:
            return False
        return type_inherits_of_type(inheriting_type.__bases__[0], base_type)
