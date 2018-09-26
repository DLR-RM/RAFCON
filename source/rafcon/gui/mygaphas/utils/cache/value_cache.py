# Copyright (C) 2016-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

from builtins import object
from builtins import str
from decimal import Context, Decimal
from hashlib import sha256
from rafcon.utils.hashable import Hashable


class ValueCache(object):
    """Cache holding values calculated from certain parameters

    This can be used to store values of variables that depend on a series of parameters. If the parameters did not
    change, the cached value can be used.
    """

    _cache = {}

    def __init__(self, precision=2):
        self.empty()
        self._context = Context(prec=precision)

    def empty(self, name=None):
        """Empty the cache

        :param name: The name of the cache that is to be deleted or None if whole cache is to be deleted
        """
        if name is None:
            self._cache = {}
        else:
            self._cache[name] = {}

    def store_value(self, name, value, parameters=None):
        """Stores the value of a certain variable

        The value of a variable with name 'name' is stored together with the parameters that were used for the
        calculation.

        :param str name: The name of the variable
        :param value: The value to be cached
        :param dict parameters: The parameters on which the value depends
        """
        if not isinstance(parameters, dict):
            raise TypeError("parameters must be a dict")
        hash = self._parameter_hash(parameters)
        if name not in self._cache:
            self._cache[name] = {}
        self._cache[name][hash.hexdigest()] = value

    def get_value(self, name, parameters=None):
        """Return the value of a cached variable if applicable

        The value of the variable 'name' is returned, if no parameters are passed or if all parameters are identical
        to the ones stored for the variable.

        :param str name: Name of teh variable
        :param dict parameters: Current parameters or None if parameters do not matter
        :return: The cached value of the variable or None if the parameters differ
        """
        if not isinstance(parameters, dict):
            raise TypeError("parameters must a dict")
        if name not in self._cache:
            return None
        hash = self._parameter_hash(parameters)
        hashdigest = hash.hexdigest()
        return self._cache[name].get(hashdigest, None)

    def _parameter_hash(self, parameters):
        self._normalize_number_values(parameters)
        hash = sha256()
        Hashable.update_hash_from_dict(hash, parameters)
        return hash

    def _normalize_number_values(self, parameters):
        """Assures equal precision for all number values"""
        for key, value in parameters.items():
            if isinstance(value, (int, float)):
                parameters[key] = str(Decimal(value).normalize(self._context))
