# Copyright (C) 2016-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from decimal import Context, Decimal


class ValueCache(object):
    """Cache holding values calculated from certain parameters

    This can be used to store values of variables that depend on a series of parameters. If the parameters did not
    change, the cached value can be used.
    """

    __value_cache = {}
    __parameter_cache = {}

    def __init__(self, precision=2):
        self.empty()
        self._context = Context(prec=precision)

    def empty(self):
        """Empty the cache

        All values are removed.
        """
        self.__value_cache = {}
        self.__parameter_cache = {}

    def store_value(self, name, value, parameters=None):
        """Stores the value of a certain variable

        The value of a variable with name 'name' is stored together with the parameters that were used for the
        calculation.

        :param str name: The name of the variable
        :param value: The value to be cached
        :param dict parameters: The parameters on which the value depends
        """
        self._normalize_number_values(parameters)
        if parameters is not None and not isinstance(parameters, dict):
            raise TypeError("parameters must be None or a dict")
        self.__value_cache[name] = value
        self.__parameter_cache[name] = parameters

    def get_value(self, name, parameters=None):
        """Return the value of a cached variable if applicable

        The value of the variable 'name' is returned, if no parameters are passed or if all parameters are identical
        to the ones stored for the variable.

        :param str name: Name of teh variable
        :param dict parameters: Current parameters or None if parameters do not matter
        :return: The cached value of the variable or None if the parameters differ
        """
        self._normalize_number_values(parameters)
        if parameters is not None and not isinstance(parameters, dict):
            raise TypeError("parameters must be None or a dict")
        if name not in self.__value_cache:
            return None
        if parameters is None:
            return self.__value_cache[name]
        for parameter, parameter_value in parameters.iteritems():
            if parameter not in self.__parameter_cache[name] or \
                    self.__parameter_cache[name][parameter] != parameter_value:
                return None
        if len(parameters) != len(self.__parameter_cache[name]):
            return None
        return self.__value_cache[name]

    def _normalize_number_values(self, parameters):
        """Assures equal precision for all number values"""
        for key, value in parameters.iteritems():
            if isinstance(value, (int, float, long)):
                parameters[key] = Decimal(value).normalize(self._context)
