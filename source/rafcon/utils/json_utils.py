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
        raise NotImplementedError()

    def to_dict(self):
        raise NotImplementedError()


class JSONObjectDecoder(JSONDecoder):

    additional_hook = None

    def __init__(self, encoding=None, object_hook=None, **kwargs):
        super(JSONObjectDecoder, self).__init__(encoding=encoding, object_hook=self.dict_to_qualified_object, **kwargs)
        self.additional_hook = object_hook

    def dict_to_qualified_object(self, dictionary):
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

            if cls is tuple:
                return tuple(dictionary['items'])

            return cls.from_dict(dictionary)

        elif '__type__' in dictionary:
            from rafcon.utils.type_helpers import convert_string_to_type
            return convert_string_to_type(dictionary['__type__'])

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

        if self.additional_hook:
            return self.additional_hook(dictionary)

        return dictionary


class JSONObjectEncoder(JSONEncoder):

    def __init__(self, **kwargs):
        super(JSONObjectEncoder, self).__init__(**kwargs)

    def default(self, obj):
        if isinstance(obj, JSONObject):
            dictionary = obj.to_dict()
            # e.g. rafcon.statemachine.states.execution_state.ExecutionState
            dictionary['__jsonqualname__'] = obj.__module__ + '.' + obj.__class__.__name__
            return dictionary

        elif isinstance(obj, type):
            return {'__type__': obj.__name__}

        else:
            return super(JSONObjectEncoder, self).default(obj)

    def encode(self, obj):
        from rafcon.utils.vividict import Vividict

        def check_for_tuples_and_vicidicts(item):
            if isinstance(item, tuple):
                dictionary = {'__jsonqualname__': '__builtin__.tuple',
                              'items': [check_for_tuples_and_vicidicts(val) for val in item]}
                return dictionary

            elif isinstance(item, dict):
                dictionary = {}
                for key, value in item.iteritems():
                    dictionary[key] = check_for_tuples_and_vicidicts(value)
                return dictionary

            elif isinstance(item, list):
                return [check_for_tuples_and_vicidicts(e) for e in item]

            elif isinstance(item, Vividict):
                dictionary = {'__jsonqualname__': item.__module__ + '.' + item.__class__.__name__}
                temp_dict = item.to_dict()
                for key, value in temp_dict.iteritems():
                    dictionary[key] = check_for_tuples_and_vicidicts(value)
                return dictionary

            return item

        return super(JSONObjectEncoder, self).encode(check_for_tuples_and_vicidicts(obj))
