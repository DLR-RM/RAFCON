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


def write_dict_to_yaml(dictionary, path, **kwargs):
    """
    Writes a dictionary to a yaml file
    :param dictionary:  the dictionary to be written
    :param path: the absolute path of the target yaml file
    :param kwargs: optional additional parameters for dumper
    """
    f = open(path, 'w')
    yaml.dump(dictionary, f, indent=4, **kwargs)
    f.close()


def load_dict_from_yaml(path):
    """
    Loads a dictionary from a yaml file
    :param path: the absolute path of the target yaml file
    :return:
    """
    f = file(path, 'r')
    dictionary = yaml.load(f)
    f.close()
    return dictionary


def write_dict_to_json(dictionary, path, **kwargs):
    """
    Write a dictionary to a json file.
    :param path: The relative path to save the dictionary to
    :param dictionary: The dictionary to get saved
    :param kwargs: optional additional parameters for dumper
    """
    f = open(path, 'w')
    json.dump(dictionary, f, indent=4)
    f.close()


def load_dict_from_json(path):
    """Loads a dictionary from a json file.

    :param path: The relative path of the json file.
    :return: The dictionary specified in the json file
    """
    f = file(path, 'r')
    result = json.load(f)
    f.close()
    return result


def create_path(path):
    """Creates a absolute path in the file system.

    :param path: The path to be created
    :return:
    """
    if not os.path.exists(path):
        os.makedirs(path)


def remove_path(path):
    """Removes an absolute path in the file system

    :param path: The path to be removed
    :return:
    """
    shutil.rmtree(path)

