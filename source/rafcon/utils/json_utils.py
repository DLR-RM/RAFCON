from json.decoder import JSONDecoder
from json.encoder import JSONEncoder


# Code partially taken from https://pymotw.com/2/json/index.html#module-json
# http://taketwoprogramming.blogspot.de/2009/06/subclassing-jsonencoder-and-jsondecoder.html

class JSONObject(object):
    """Interface class

    Classes deriving from this class can be stored as JSON objects.
    """

    @classmethod
    def from_dict(cls, dictionary):
        """Abstract method

        This method must be implemented by the deriving classes. It must return an object of type cls, created from the
        parameters defined in dictionary. The type of cls is the type of the class the method is called on.

        :param dict dictionary: A Python dict with all parameters needed for creating an object of type cls
        :return: An instance of cls
        :rtype: cls
        """
        raise NotImplementedError()

    def to_dict(self):
        """Abstract method

        This method must be implemented by the deriving classes. It must return a Python dict with all parameters of
        self, which are needed to create a copy of self.

        :return: A Python dict with all needed parameters of self
        :rtype: dict
        """
        raise NotImplementedError()


class JSONObjectDecoder(JSONDecoder):
    """Custom JSON decoder class especially for state machines

    This JSON decoder class inherits from the basic JSON decoder class. It can decode all classes deriving from
    JSONObject. In addition, tuples (encoded by :py:class:`JSONObjectEncoder`) are maintained and type objects (e.g.
    int, object, float) are handled. Finally, it is tried to convert dictionary keys to integers.
    """

    additional_hook = None

    def __init__(self, encoding=None, object_hook=None, **kwargs):
        super(JSONObjectDecoder, self).__init__(encoding=encoding, object_hook=self._dict_to_qualified_object, **kwargs)
        self.additional_hook = object_hook

    def _dict_to_qualified_object(self, dictionary):
        # Handle classes deriving from JSONObject and tuples
        if '__jsonqualname__' in dictionary:
            # e.g. rafcon.statemachine.states.execution_state.ExecutionState
            qualified_name = dictionary.pop('__jsonqualname__')
            parts = qualified_name.split('.')
            module_name = ".".join(parts[:-1])
            # First ensure, that the module is imported
            cls = __import__(module_name)
            # Find nested class
            for comp in parts[1:]:
                cls = getattr(cls, comp)

            # Maintain tuples instead of converting them to a list
            if cls is tuple:
                return tuple(dictionary['items'])

            # from_dict must be implemented by classes deriving from JSONObject
            return cls.from_dict(dictionary)

        # Handle type objects
        elif '__type__' in dictionary:
            from rafcon.utils.type_helpers import convert_string_to_type
            return convert_string_to_type(dictionary['__type__'])

        # Try to convert dictionary keys to integers
        elif isinstance(dictionary, dict):
            # Converts keys to integers, where possible
            temp_dictionary = {}
            for key, value in dictionary.iteritems():
                try:
                    key = int(key)
                except ValueError:
                    pass
                temp_dictionary[key] = value
            dictionary = temp_dictionary

        # If an additional converter function was passes to the constructor, call it now
        if self.additional_hook:
            return self.additional_hook(dictionary)

        return dictionary


class JSONObjectEncoder(JSONEncoder):
    """Custom JSON encoder class especially for state machines

    This JSON encoder class inherits from the basic JSON encoder class. It can encode all classes deriving from
    JSONObject. In addition, tuples (encoded by :py:class:`JSONObjectEncoder`) are maintained and type objects (e.g.
    int, object, float) are handled. Finally, it is tried to convert dictionary keys to integers.
    """

    def default(self, obj):
        """This method is called after all base class encoding logic

        Here, additional conversions of objects to dictionaries can be defined. Both JSONObject and type objects are
        treated.

        :param obj: Object to be converted to a dictionary
        :return: Dictionary representing the given object
        :rtype: dict
        """
        if isinstance(obj, JSONObject):
            # to_dict must be implemented by classes deriving from JSONObject
            dictionary = obj.to_dict()
            # e.g. rafcon.statemachine.states.execution_state.ExecutionState
            dictionary['__jsonqualname__'] = obj.__module__ + '.' + obj.__class__.__name__
            return dictionary

        elif isinstance(obj, type):
            return {'__type__': obj.__name__}

        else:
            return super(JSONObjectEncoder, self).default(obj)

    def encode(self, obj):
        """This method is called before any object conversion of the inherited class

        The method checks for (nested) tuples and Vividicts and converts them appropriately.

        :param obj: The object to be encoded
        :return: Encoded object
        :rtype: dict
        """
        from rafcon.utils.vividict import Vividict

        # Recursive check for tuples and Vividicts (within tuples, dicts and lists)
        def check_for_tuples_and_vividicts(item):
            if isinstance(item, tuple):
                dictionary = {'__jsonqualname__': '__builtin__.tuple',
                              'items': [check_for_tuples_and_vividicts(val) for val in item]}
                return dictionary

            elif isinstance(item, dict):
                dictionary = {}
                for key, value in item.iteritems():
                    dictionary[key] = check_for_tuples_and_vividicts(value)
                return dictionary

            elif isinstance(item, list):
                return [check_for_tuples_and_vividicts(e) for e in item]

            elif isinstance(item, Vividict):
                dictionary = {'__jsonqualname__': item.__module__ + '.' + item.__class__.__name__}
                temp_dict = item.to_dict()
                for key, value in temp_dict.iteritems():
                    dictionary[key] = check_for_tuples_and_vividicts(value)
                return dictionary

            return item

        return super(JSONObjectEncoder, self).encode(check_for_tuples_and_vividicts(obj))
