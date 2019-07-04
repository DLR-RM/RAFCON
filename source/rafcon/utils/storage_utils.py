# Copyright (C) 2015-2017 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: storage_utils
   :synopsis: Utility methods for storing and loading files from disk (supports several formats)

"""

import json
import yaml
from time import gmtime, strftime, strptime, mktime

from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder

substitute_modules = {
    # backward compatibiliy (remove in next minor release): state elements
    'rafcon.statemachine.data_flow.DataFlow': 'rafcon.core.state_elements.data_flow.DataFlow',
    'rafcon.statemachine.data_port.DataPort': 'rafcon.core.state_elements.data_port.DataPort',
    'rafcon.statemachine.data_port.InputDataPort': 'rafcon.core.state_elements.data_port.InputDataPort',
    'rafcon.statemachine.data_port.OutputDataPort': 'rafcon.core.state_elements.data_port.OutputDataPort',
    'rafcon.statemachine.scope.ScopedData': 'rafcon.core.state_elements.scope.ScopedData',
    'rafcon.statemachine.scope.ScopedVariable': 'rafcon.core.state_elements.scope.ScopedVariable',
    'rafcon.statemachine.state_element.StateElement': 'rafcon.core.state_elements.state_element.StateElement',
    'rafcon.statemachine.transition.Transition': 'rafcon.core.state_elements.transition.Transition',
    'rafcon.statemachine.outcome.Outcome': 'rafcon.core.state_elements.logical_port.Outcome',
    # state elements: statemachine => core
    'rafcon.statemachine.state_elements.data_flow.DataFlow': 'rafcon.core.state_elements.data_flow.DataFlow',
    'rafcon.statemachine.state_elements.data_port.DataPort': 'rafcon.core.state_elements.data_port.DataPort',
    'rafcon.statemachine.state_elements.data_port.InputDataPort': 'rafcon.core.state_elements.data_port.InputDataPort',
    'rafcon.statemachine.state_elements.data_port.OutputDataPort': 'rafcon.core.state_elements.data_port.OutputDataPort',
    'rafcon.statemachine.state_elements.scope.ScopedData': 'rafcon.core.state_elements.scope.ScopedData',
    'rafcon.statemachine.state_elements.scope.ScopedVariable': 'rafcon.core.state_elements.scope.ScopedVariable',
    'rafcon.statemachine.state_elements.state_element.StateElement': 'rafcon.core.state_elements.state_element.StateElement',
    'rafcon.statemachine.state_elements.transition.Transition': 'rafcon.core.state_elements.transition.Transition',
    'rafcon.statemachine.state_elements.outcome.Outcome': 'rafcon.core.state_elements.logical_port.Outcome',
    # states: statemachine => core
    'rafcon.statemachine.states.barrier_concurrency_state.BarrierConcurrencyState':
        'rafcon.core.states.barrier_concurrency_state.BarrierConcurrencyState',
    'rafcon.statemachine.states.barrier_concurrency_state.DeciderState':
        'rafcon.core.states.barrier_concurrency_state.DeciderState',
    'rafcon.statemachine.states.concurrency_state.ConcurrencyState': 'rafcon.core.states.concurrency_state.ConcurrencyState',
    'rafcon.statemachine.states.container_state.ContainerState': 'rafcon.core.states.container_state.ContainerState',
    'rafcon.statemachine.states.execution_state.ExecutionState': 'rafcon.core.states.execution_state.ExecutionState',
    'rafcon.statemachine.states.hierarchy_state.HierarchyState': 'rafcon.core.states.hierarchy_state.HierarchyState',
    'rafcon.statemachine.states.library_state.LibraryState': 'rafcon.core.states.library_state.LibraryState',
    'rafcon.statemachine.states.preemptive_concurrency_state.PreemptiveConcurrencyState': 'rafcon.core.states.preemptive_concurrency_state.PreemptiveConcurrencyState',
    'rafcon.statemachine.states.state.State': 'rafcon.core.states.state.State',
    # logical_port
    'rafcon.core.state_elements.outcome.Outcome': 'rafcon.core.state_elements.logical_port.Outcome'
}


TIME_STRING_FORMAT = "%Y-%m-%d %H:%M:%S"


def get_current_time_string():
    return strftime(TIME_STRING_FORMAT, gmtime())


def get_float_time_for_string(string):
    return mktime(strptime(string, TIME_STRING_FORMAT))


def get_time_string_for_float(seconds):
    return strftime(TIME_STRING_FORMAT, gmtime(seconds))


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
    result_string = json.dumps(dictionary, cls=JSONObjectEncoder,
                               indent=4, separators=(', ', ': '), builtins_str="__builtin__", sort_keys=True,
                               check_circular=False, **kwargs)
    with open(path, 'w') as f:
        # We cannot write directly to the file, as otherwise the 'encode' method wouldn't be called
        f.write(result_string)


def load_objects_from_json(path, as_dict=False):
    """Loads a dictionary from a json file.

    :param path: The relative path of the json file.
    :return: The dictionary specified in the json file
    """
    f = open(path, 'r')
    if as_dict:
        result = json.load(f)
    else:
        result = json.load(f, cls=JSONObjectDecoder, substitute_modules=substitute_modules)
    f.close()
    return result
