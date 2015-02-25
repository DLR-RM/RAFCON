import yaml


class Vividict(yaml.YAMLObject, dict):
    """
    A class which inherits from dict and can store an element for an arbitrary nested key. The single elements of the
    key do not have to exist beforehand.
    """
    def __init__(self):
        pass

    def __missing__(self, key):
        """
        The main function of this class. If a key is missing it creates a new Vividict on the fly.
        :param key: the missing key for a value to be stored
        :return: the new value for the specified key
        """
        value = self[key] = type(self)()
        return value

    def set_dict(self, new_dict):
        """
        Sets the dictionary of the Vividict.
        :param new_dict: The dict that will be added to the own dict
        :return:
        """
        for key, value in new_dict.iteritems():
            self[key] = value

    yaml_tag = u'!Vividict'

    @classmethod
    def to_yaml(cls, dumper, data):
        dict_to_save = {}
        for key, value in data.iteritems():
            dict_to_save[key] = value
        dict_representation = {
            'vivid_dict_content': dict_to_save
        }
        node = dumper.represent_mapping(u'!Vividict', dict_representation)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)
        initial_dict = dict_representation['vivid_dict_content']
        result_vivid_dict = Vividict()
        result_vivid_dict.set_dict(initial_dict)
        return result_vivid_dict