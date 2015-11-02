import yaml


class Vividict(yaml.YAMLObject, dict):
    """
    A class which inherits from dict and can store an element for an arbitrary nested key. The single elements of the
    key do not have to exist beforehand.
    """
    def __init__(self, dictionary=None):
        if dictionary:
            self.set_dict(dictionary)

    def __missing__(self, key):
        """
        The main function of this class. If a key is missing it creates a new Vividict on the fly.
        :param key: the missing key for a value to be stored
        :return: the new value for the specified key
        """
        value = self[key] = type(self)()
        return value

    def set_dict(self, new_dict):
        """Sets the dictionary of the Vividict

        The method is able to handle nested dictionaries, by calling the method recursively.

        :param new_dict: The dict that will be added to the own dict
        """
        for key, value in new_dict.iteritems():
            if isinstance(value, dict):
                self[str(key)] = Vividict(value)
            else:
                self[str(key)] = value

    yaml_tag = u'!Vividict'

    @classmethod
    def to_yaml(cls, dumper, data):
        from numpy import ndarray
        dict_to_save = {}

        def np_to_native(np_val):
            """Recursively convert numpy values to native Python values

            - Converts matrices to lists
            - Converts numpy.dtypes to float/int etc
            :param np_val: value to convert
            :return: value as native Python value
            """
            if isinstance(np_val, dict):
                for key, value in np_val.iteritems():
                    np_val[key] = np_to_native(value)
            if isinstance(np_val, ndarray):
                np_val = np_val.tolist()
            if isinstance(np_val, (list, tuple)):
                native_list = [np_to_native(val) for val in np_val]
                if isinstance(np_val, tuple):
                    return tuple(native_list)
                return native_list
            if not hasattr(np_val, 'dtype'):  # Nothing to convert
                return np_val
            return np_val.item()  # Get the gloat/int etc value

        for key, value in data.iteritems():
            # Remove old entries from meta data
            # TODO: remove this sometime in the future
            if key not in ['from_pos_x', 'from_pos_y', 'to_pos_x', 'to_pos_y', 'id', 'width', 'height',
                           'connector_pos', 'connector_radius', 'inner_pos', 'rect_height', 'rect_width',
                           'outcome_radius', 'pos_x', 'pos_y', 'width)', 'resize_length']:
                # Convert numpy values to native Python values
                value = np_to_native(value)
                dict_to_save[key] = value
        node = dumper.represent_mapping(cls.yaml_tag, dict_to_save)
        return node

    @classmethod
    def from_yaml(cls, loader, node):
        dict_representation = loader.construct_mapping(node, deep=True)

        # Keep this check only for backwards compatability, initial_dict = dict_representation is sufficient
        if 'vivid_dict_content' in dict_representation:
            initial_dict = dict_representation['vivid_dict_content']
        else:
            initial_dict = dict_representation
        result_vivid_dict = Vividict()
        result_vivid_dict.set_dict(initial_dict)
        return result_vivid_dict