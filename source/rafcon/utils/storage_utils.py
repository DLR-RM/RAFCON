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
import builtins
from time import gmtime, strftime

from jsonconversion.decoder import JSONObjectDecoder
from jsonconversion.encoder import JSONObjectEncoder
from rafcon.core.config import global_config

# For backward compatibility of old state machines that have these strings in their __jsonqualname__ field in .json
substitute_modules = {
    # after introducing state elements
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
    'rafcon.core.state_elements.outcome.Outcome': 'rafcon.core.state_elements.logical_port.Outcome'
}

TIME_STRING_FORMAT = "%Y-%m-%d %H:%M:%S"

# We define the safe types for loading state machines (usually used for data_port types in
# the __type__ field)
RAFCON_ALLOWED_TYPES = {
    # python3 builtins common types that are considered safe
    'builtins.int': builtins.int,
    'builtins.str': builtins.str,
    'builtins.float': builtins.float,
    'builtins.bool': builtins.bool,
    'builtins.list': builtins.list,
    'builtins.dict': builtins.dict,
    'builtins.tuple': builtins.tuple,
    'builtins.set': builtins.set,
    'builtins.bytes': builtins.bytes,
    'builtins.range': builtins.range,
    'builtins.complex': builtins.complex,
    'builtins.bytearray': builtins.bytearray,
    'builtins.object': builtins.object,
    # currently, .json files are still saved with python2 __builtin__ convention
    '__builtin__.int': builtins.int,
    '__builtin__.str': builtins.str,
    '__builtin__.float': builtins.float,
    '__builtin__.bool': builtins.bool,
    '__builtin__.list': builtins.list,
    '__builtin__.dict': builtins.dict,
    '__builtin__.tuple': builtins.tuple,
    '__builtin__.set': builtins.set,
    '__builtin__.bytes': builtins.bytes,
    '__builtin__.range': builtins.range,
    '__builtin__.complex': builtins.complex,
    '__builtin__.bytearray': builtins.bytearray,
    '__builtin__.object': builtins.object,
}


def get_allowed_classes():
    '''For loading state machines from .json we define the allowed classes to provide some security
    with jsonconversion which reconstructs objects from the __jsonqualname__ field
    '''
    # Importing inside the function avoids module-level circular imports
    from rafcon.core.state_elements.data_flow import DataFlow
    from rafcon.core.state_elements.data_port import DataPort, InputDataPort, OutputDataPort
    from rafcon.core.state_elements.logical_port import Income, Outcome
    from rafcon.core.state_elements.scope import ScopedData, ScopedVariable
    from rafcon.core.state_elements.state_element import StateElement
    from rafcon.core.state_elements.transition import Transition
    from rafcon.core.states.barrier_concurrency_state import BarrierConcurrencyState
    from rafcon.core.states.barrier_concurrency_state import DeciderState
    from rafcon.core.states.concurrency_state import ConcurrencyState
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.hierarchy_state import HierarchyState
    from rafcon.core.states.library_state import LibraryState
    from rafcon.core.states.preemptive_concurrency_state import PreemptiveConcurrencyState
    from rafcon.core.states.state import State
    from numpy import ndarray

    RAFCON_ALLOWED_CLASSES = {
        # RAFCON classes that need to be reconstructed
        'rafcon.core.state_elements.data_flow.DataFlow': DataFlow,
        'rafcon.core.state_elements.data_port.DataPort': DataPort,
        'rafcon.core.state_elements.data_port.InputDataPort': InputDataPort,
        'rafcon.core.state_elements.data_port.OutputDataPort': OutputDataPort,
        'rafcon.core.state_elements.logical_port.Outcome': Outcome,
        'rafcon.core.state_elements.logical_port.Income': Income,
        'rafcon.core.state_elements.scope.ScopedData': ScopedData,
        'rafcon.core.state_elements.scope.ScopedVariable': ScopedVariable,
        'rafcon.core.state_elements.state_element.StateElement': StateElement,
        'rafcon.core.state_elements.transition.Transition': Transition,
        'rafcon.core.states.barrier_concurrency_state.BarrierConcurrencyState': BarrierConcurrencyState,
        'rafcon.core.states.barrier_concurrency_state.DeciderState': DeciderState,
        'rafcon.core.states.concurrency_state.ConcurrencyState':ConcurrencyState,
        'rafcon.core.states.container_state.ContainerState': ContainerState,
        'rafcon.core.states.execution_state.ExecutionState': ExecutionState,
        'rafcon.core.states.hierarchy_state.HierarchyState': HierarchyState,
        'rafcon.core.states.library_state.LibraryState': LibraryState,
        'rafcon.core.states.preemptive_concurrency_state.PreemptiveConcurrencyState': PreemptiveConcurrencyState,
        'rafcon.core.states.state.State': State,
        # python3 builtins classes that need to be reconstructed (e.g. for meta_data.json)
        'builtins.tuple': builtins.tuple,
        'builtins.set': builtins.set,
        # currently, .json files are still saved with python2 __builtin__ convention
        '__builtin__.tuple': builtins.tuple,
        '__builtin__.set': builtins.set,
        # for legacy support of jsonconversion encodings of numpy.ndarray
        'numpy.ndarray': ndarray,
    }
    return RAFCON_ALLOWED_CLASSES


def get_current_time_string():
    return strftime(TIME_STRING_FORMAT, gmtime())


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
        # get config option to allow untrusted types
        allow_untrusted_types = global_config.get_config_value("ALLOW_UNTRUSTED_DATA_TYPES", False)
        try:
            result = json.load(
                f,
                cls=JSONObjectDecoder,
                allowed_classes=get_allowed_classes(),
                allowed_types=RAFCON_ALLOWED_TYPES,
                allow_untrusted_types=allow_untrusted_types,
                substitute_modules=substitute_modules)
        except ValueError as e:
            raise ValueError(
                "Failed to load state machine! Enable import of untrusted data types in the "\
                "core configuration with the ALLOW_UNTRUSTED_DATA_TYPES field."
            ) from e
    f.close()
    return result
