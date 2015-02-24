import yaml


class Vividict(yaml.YAMLObject, dict):

    def __init__(self):
        pass

    def __missing__(self, key):
        value = self[key] = type(self)()
        return value

    def set_initial_dict(self, initial_dict):
        for key, value in initial_dict.iteritems():
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
        result_vivid_dict.set_initial_dict(initial_dict)
        return result_vivid_dict