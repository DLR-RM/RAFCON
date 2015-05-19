"""
.. module:: storage_utils
   :platform: Unix, Windows
   :synopsis: Utility methods for storing and loading files from disk (supports several formats)

.. moduleauthor:: Sebastian Brunner


"""

import os
import shutil
import json

import yaml


class StorageUtils(object):
    """
    A class which holds different load and save utilities for yaml and json format.
    """
    def __init__(self, base_path=None):
        self.base_path = base_path
        pass

    def save_object_to_yaml_rel(self, object, rel_path):
        """
        Saves an object, which inherits from yaml.YAMLObject, to yaml file
        :param object: the object to be saved
        :param rel_path: the relative path of the target yaml file
        :return:
        """
        f = open(os.path.join(self.base_path, rel_path), 'w')
        yaml.dump(object, f, indent=4)
        f.close()

    def save_object_to_yaml_abs(self, object, abs_path):
        """
        Saves an object, which inherits from yaml.YAMLObject, to yaml file
        :param object: the object to be saved
        :param abs_path: the absolute path of the target yaml file
        :return:
        """
        f = open(abs_path, 'w')
        yaml.dump(object, f, indent=4)
        f.close()

    def load_object_from_yaml_rel(self, rel_path):
        """
        Loads an object, which inherits from yaml.YAMLObject, from a yaml file
        :param object: the object to be saved
        :param abs_path: the relative path of the target yaml file
        :return:
        """
        stream = file(os.path.join(self.base_path, rel_path), 'r')
        state = yaml.load(stream)
        return state

    def load_object_from_yaml_abs(self, abs_path):
        """
        Loads an object, which inherits from yaml.YAMLObject, from a yaml file
        :param object: the object to be saved
        :param abs_path: the absolute path of the target yaml file
        :return:
        """
        stream = file(abs_path, 'r')
        state = yaml.load(stream)
        return state

    def write_dict_to_yaml(self, dict_to_write, path, **kwords):
        """
        Writes a dictionary to a yaml file
        :param dict_to_write:  the dictionary to be written
        :param path: the absolute path of the target yaml file
        :return:
        """
        f = open(path, 'w')
        yaml.dump(dict_to_write, f, indent=4, **kwords)
        f.close()

    def load_dict_from_yaml(self, path):
        """
        Loads a dictionary from a yaml file
        :param path: the absolute path of the target yaml file
        :return:
        """
        stream = file(path, 'r')
        yaml_object = yaml.load(stream)
        return yaml_object

    def write_dict_to_json(self, rel_path, tmp_dict):
        """
        Write a dictionary to a json file.
        :param rel_path: The relative path to save the dictionary to
        :param tmp_dict: The dictionary to get saved
        :return:
        """
        f = open(os.path.join(self.base_path, rel_path), 'w')
        json.dump(tmp_dict, f, indent=4)
        f.close()

    def load_dict_from_json(self, rel_path):
        """
        Loads a dictionary from a json file.
        :param rel_path: The relative path of the json file.
        :return: The dictionary specified in the json file
        """
        f = open(os.path.join(self.base_path, rel_path), 'r')
        result = json.load(f)
        f.close()
        return result

    def create_path(self, path):
        """ Creats a absolute path in the file system.

        :param path: The path to be created
        :return:
        """
        if not self.exists_path(path):
            os.makedirs(path)

    def remove_path(self, path):
        """ Removes an absolute path in the file system

        :param path: The path to be removed
        :return:
        """
        shutil.rmtree(path)

    def _remove_file(self, path):
        """
        Removes a file in the file system.
        :param path: The absolute path of the file to be removed.
        :return:
        """
        os.remove(path)

    def exists_path(self, path):
        """
        Checks if a certain path exists in the file system.
        :param path: The path to be checked
        :return:
        """
        return os.path.exists(path)

