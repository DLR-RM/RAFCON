"""
.. module:: storage_utils
   :platform: Unix, Windows
   :synopsis: Utility methods for storing and loading files from disk (supports several formats)

.. moduleauthor:: Sebastian Brunner


"""

import json
import yaml
from time import gmtime, strftime

from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder

substitute_modules = {
    'rafcon.statemachine.data_flow.DataFlow': 'rafcon.statemachine.state_elements.data_flow.DataFlow',
    'rafcon.statemachine.data_port.DataPort': 'rafcon.statemachine.state_elements.data_port.DataPort',
    'rafcon.statemachine.data_port.InputDataPort': 'rafcon.statemachine.state_elements.data_port.InputDataPort',
    'rafcon.statemachine.data_port.OutputDataPort': 'rafcon.statemachine.state_elements.data_port.OutputDataPort',
    'rafcon.statemachine.scope.ScopedData': 'rafcon.statemachine.state_elements.scope.ScopedData',
    'rafcon.statemachine.scope.ScopedVariable': 'rafcon.statemachine.state_elements.scope.ScopedVariable',
    'rafcon.statemachine.state_element.StateElement': 'rafcon.statemachine.state_elements.state_element.StateElement',
    'rafcon.statemachine.transition.Transition': 'rafcon.statemachine.state_elements.transition.Transition',
    'rafcon.statemachine.outcome.Outcome': 'rafcon.statemachine.state_elements.outcome.Outcome',
}


def get_current_time_string():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())


def write_dict_to_yaml(dictionary, path, **kwargs):
    """
    Writes a dictionary to a yaml file
    :param dictionary:  the dictionary to be written
    :param path: the absolute path of the target yaml file
    :param kwargs: optional additional parameters for dumper
    """
    with open(path, 'w') as f:
        yaml.dump(dictionary, f, indent=4, **kwargs)


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
    result_string = json.dumps(dictionary, cls=JSONObjectEncoder, nested_jsonobjects=False,
                               indent=4, check_circular=False, sort_keys=True, **kwargs)
    with open(path, 'w') as f:
        # We cannot write directly to the file, as otherwise the 'encode' method wouldn't be called
        f.write(result_string)


def load_objects_from_json(path, as_dict=False):
    """Loads a dictionary from a json file.

    :param path: The relative path of the json file.
    :return: The dictionary specified in the json file
    """
    f = file(path, 'r')
    if as_dict:
        result = json.load(f)
    else:
        result = json.load(f, cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    f.close()
    return result
