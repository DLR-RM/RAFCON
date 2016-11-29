
class ValueCache(object):
    """Cache holding values calculated from certain parameters

    This can be used to store values of variables that depend on a series of parameters. If the parameters did not
    change, the cached value can be used.
    """

    __value_cache = {}
    __parameter_cache = {}

    def __init__(self):
        self.empty()

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
