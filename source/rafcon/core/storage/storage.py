# Copyright (C) 2015-2018 DLR
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
.. module:: storage
   :synopsis: Helper functions to store a state machine in the local file system and load it from there

"""

from weakref import ref
from future.utils import string_types
from builtins import str
import os
import re
import math
import shutil
import glob
import copy
import yaml
import warnings
from distutils.version import StrictVersion

import rafcon

from rafcon.utils.filesystem import read_file, write_file
from rafcon.utils import storage_utils
from rafcon.utils import log
from rafcon.utils.timer import measure_time

from rafcon.core.custom_exceptions import LibraryNotFoundException
from rafcon.core.constants import DEFAULT_SCRIPT_PATH
from rafcon.core.config import global_config
from rafcon.core.state_machine import StateMachine

logger = log.get_logger(__name__)

LIBRARY_NOT_FOUND_DUMMY_STATE_NAME = "LIBRARY NOT FOUND DUMMY STATE"

#: File names for various purposes
FILE_NAME_META_DATA = 'meta_data.json'
FILE_NAME_META_DATA_OLD = 'gui_Gtk.json'
FILE_NAME_CORE_DATA = 'core_data.json'
FILE_NAME_CORE_DATA_OLD = 'meta.json'
SCRIPT_FILE = 'script.py'
SEMANTIC_DATA_FILE = 'semantic_data.json'
STATEMACHINE_FILE = 'statemachine.json'
STATEMACHINE_FILE_OLD = 'statemachine.yaml'
ID_NAME_DELIMITER = "_"

REPLACED_CHARACTERS_FOR_NO_OS_LIMITATION = {'/': '', r'\0': '', '<': '', '>': '', ':': '_',
                                            '\\': '', '|': '_', '?': '', '*': '_'}

# clean the DEFAULT_SCRIPT_PATH folder at each program start
if os.path.exists(DEFAULT_SCRIPT_PATH):
    files = glob.glob(os.path.join(DEFAULT_SCRIPT_PATH, "*"))
    for f in files:
        shutil.rmtree(f)


def remove_obsolete_folders(states, path):
    """Removes obsolete state machine folders

    This function removes all folders in the file system folder `path` that do not belong to the states given by
    `states`.
    
    :param list states: the states that should reside in this very folder
    :param str path: the file system path to be checked for valid folders
    """
    elements_in_folder = os.listdir(path)
    # find all state folder elements in system path
    state_folders_in_file_system = []
    for folder_name in elements_in_folder:
        if os.path.exists(os.path.join(path, folder_name, FILE_NAME_CORE_DATA)) or \
                os.path.exists(os.path.join(path, folder_name, FILE_NAME_CORE_DATA_OLD)):
            state_folders_in_file_system.append(folder_name)

    # remove elements used by existing states and storage format
    for state in states:
        storage_folder_for_state = get_storage_id_for_state(state)
        if storage_folder_for_state in state_folders_in_file_system:
            state_folders_in_file_system.remove(storage_folder_for_state)

    # remove the remaining state folders
    for folder_name in state_folders_in_file_system:
        shutil.rmtree(os.path.join(path, folder_name))


def clean_path_from_deprecated_naming(base_path):
    """ Checks if the base path includes deprecated characters/format and returns corrected version

    The state machine folder name should be according the universal RAFCON path format. In case the state machine path
    is inside a mounted library_root_path also the library_path has to have this format. The library path is a
    partial path of the state machine path. This rules are followed to always provide secure paths for RAFCON and all
    operating systems.

    :param base_path:
    :return: cleaned base_path
    :rtype: str
    """
    def warning_logger_message(insert_string):
        not_allowed_characters = "'" + "', '".join(REPLACED_CHARACTERS_FOR_NO_OS_LIMITATION.keys()) + "'"
        warnings.warn("{1} not allowed in {2} of {0}".format(base_path, not_allowed_characters, insert_string),
                      log.RAFCONDeprecationWarning)
    from rafcon.core.singleton import library_manager
    if library_manager.is_os_path_within_library_root_paths(base_path):
        library_path, library_name = library_manager.get_library_path_and_name_for_os_path(base_path)
        clean_library_path = clean_path(library_path)
        clean_library_name = clean_path(library_name)
        if library_name != clean_library_name or library_path != clean_library_path:
            warning_logger_message("library path")
        library_root_key = library_manager._get_library_root_key_for_os_path(base_path)
        library_root_path = library_manager._library_root_paths[library_root_key]
        clean_base_path = os.path.join(library_root_path, clean_library_path, clean_library_name)
    else:
        path_elements = base_path.split(os.path.sep)
        state_machine_folder_name = base_path.split(os.path.sep)[-1]
        path_elements[-1] = clean_path(state_machine_folder_name)
        if not state_machine_folder_name == path_elements[-1]:
            warning_logger_message("state machine folder name")
        clean_base_path = os.path.sep.join(path_elements)
    return clean_base_path


def clean_path(base_path):
    """
    This function cleans a file system path in terms of removing all not allowed characters of each path element.
    A path element is an element of a path between the path separator of the operating system.
    
    :param base_path: the path to be cleaned
    :return: the clean path
    """
    path_elements = base_path.split(os.path.sep)
    reduced_path_elements = [clean_path_element(elem, max_length=255) for elem in path_elements]
    if not all(path_elements[i] == elem for i, elem in enumerate(reduced_path_elements)):
        # logger.info("State machine storage path is reduced")
        base_path = os.path.sep.join(reduced_path_elements)
    return base_path


def save_state_machine_to_path(state_machine, base_path, delete_old_state_machine=False, as_copy=False):
    """Saves a state machine recursively to the file system

    The `as_copy` flag determines whether the state machine is saved as copy. If so (`as_copy=True`), some state
    machine attributes will be left untouched, such as the `file_system_path` or the `dirty_flag`.

    :param rafcon.core.state_machine.StateMachine state_machine: the state_machine to be saved
    :param str base_path: base_path to which all further relative paths refers to
    :param bool delete_old_state_machine: Whether to delete any state machine existing at the given path
    :param bool as_copy: Whether to use a copy storage for the state machine
    """
    # warns the user in the logger when using deprecated names
    clean_path_from_deprecated_naming(base_path)

    state_machine.acquire_modification_lock()
    try:
        root_state = state_machine.root_state

        # clean old path first
        if delete_old_state_machine:
            if os.path.exists(base_path):
                shutil.rmtree(base_path)

        # Ensure that path is existing
        if not os.path.exists(base_path):
            os.makedirs(base_path)

        old_update_time = state_machine.last_update
        state_machine.last_update = storage_utils.get_current_time_string()
        state_machine_dict = state_machine.to_dict()
        storage_utils.write_dict_to_json(state_machine_dict, os.path.join(base_path, STATEMACHINE_FILE))

        # set the file_system_path of the state machine
        if not as_copy:
            state_machine.file_system_path = copy.copy(base_path)
        else:
            state_machine.last_update = old_update_time

        # add root state recursively
        remove_obsolete_folders([root_state], base_path)
        save_state_recursively(root_state, base_path, "", as_copy)

        if state_machine.marked_dirty and not as_copy:
            state_machine.marked_dirty = False
        logger.debug("State machine with id {0} was saved at {1}".format(state_machine.state_machine_id, base_path))
    except Exception:
        raise
    finally:
        state_machine.release_modification_lock()


def save_script_file_for_state_and_source_path(state, state_path_full, as_copy=False):
    """Saves the script file for a state to the directory of the state.

    The script name will be set to the SCRIPT_FILE constant.

    :param state: The state of which the script file should be saved
    :param str state_path_full: The path to the file system storage location of the state
    :param bool as_copy: Temporary storage flag to signal that the given path is not the new file_system_path
    """
    from rafcon.core.states.execution_state import ExecutionState
    if isinstance(state, ExecutionState):
        source_script_file = os.path.join(state.script.path, state.script.filename)
        destination_script_file = os.path.join(state_path_full, SCRIPT_FILE)

        try:
            write_file(destination_script_file, state.script_text)
        except Exception:
            logger.exception("Storing of script file failed: {0} -> {1}".format(state.get_path(),
                                                                                destination_script_file))
            raise

        if not source_script_file == destination_script_file and not as_copy:
            state.script.filename = SCRIPT_FILE
            state.script.path = state_path_full


def save_semantic_data_for_state(state, state_path_full):
    """Saves the semantic data in a separate json file.

    :param state: The state of which the script file should be saved
    :param str state_path_full: The path to the file system storage location of the state
    """

    destination_script_file = os.path.join(state_path_full, SEMANTIC_DATA_FILE)

    if state.semantic_data:
        try:
            storage_utils.write_dict_to_json(state.semantic_data, destination_script_file)
        except IOError:
            logger.exception("Storing of semantic data for state {0} failed! Destination path: {1}".
                             format(state.get_path(), destination_script_file))
            raise


def save_state_recursively(state, base_path, parent_path, as_copy=False):
    """Recursively saves a state to a json file

    It calls this method on all its substates.

    :param state: State to be stored
    :param base_path: Path to the state machine
    :param parent_path: Path to the parent state
    :param bool as_copy: Temporary storage flag to signal that the given path is not the new file_system_path
    :return:
    """
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.container_state import ContainerState

    state_path = os.path.join(parent_path, get_storage_id_for_state(state))
    state_path_full = os.path.join(base_path, state_path)
    if not os.path.exists(state_path_full):
        os.makedirs(state_path_full)

    storage_utils.write_dict_to_json(state, os.path.join(state_path_full, FILE_NAME_CORE_DATA))
    if not as_copy:
        state.file_system_path = state_path_full

    if isinstance(state, ExecutionState):
        save_script_file_for_state_and_source_path(state, state_path_full, as_copy)

    save_semantic_data_for_state(state, state_path_full)

    # create yaml files for all children
    if isinstance(state, ContainerState):
        remove_obsolete_folders(state.states.values(), os.path.join(base_path, state_path))
        for state in state.states.values():
            save_state_recursively(state, base_path, state_path, as_copy)


@measure_time
def load_state_machine_from_path(base_path, state_machine_id=None):
    """Loads a state machine from the given path

    :param base_path: An optional base path for the state machine.
    :return: a tuple of the loaded container state, the version of the state and the creation time
    :raises ValueError: if the provided path does not contain a valid state machine
    """
    logger.debug("Loading state machine from path {0}...".format(base_path))

    state_machine_file_path = os.path.join(base_path, STATEMACHINE_FILE)
    state_machine_file_path_old = os.path.join(base_path, STATEMACHINE_FILE_OLD)

    # was the root state specified as state machine base_path to load from?
    if not os.path.exists(state_machine_file_path) and not os.path.exists(state_machine_file_path_old):

        # catch the case that a state machine root file is handed
        if os.path.exists(base_path) and os.path.isfile(base_path):
            base_path = os.path.dirname(base_path)
            state_machine_file_path = os.path.join(base_path, STATEMACHINE_FILE)
            state_machine_file_path_old = os.path.join(base_path, STATEMACHINE_FILE_OLD)

        if not os.path.exists(state_machine_file_path) and not os.path.exists(state_machine_file_path_old):
            raise ValueError("Provided path doesn't contain a valid state machine: {0}".format(base_path))

    state_machine_dict = storage_utils.load_objects_from_json(state_machine_file_path)
    if 'used_rafcon_version' in state_machine_dict:
        previously_used_rafcon_version = StrictVersion(state_machine_dict['used_rafcon_version']).version
        active_rafcon_version = StrictVersion(rafcon.__version__).version

        rafcon_newer_than_sm_version = "You are trying to load a state machine that was stored with an older " \
                                       "version of RAFCON ({0}) than the one you are using ({1}).".format(
                                        state_machine_dict['used_rafcon_version'], rafcon.__version__)
        rafcon_older_than_sm_version = "You are trying to load a state machine that was stored with an newer " \
                                       "version of RAFCON ({0}) than the one you are using ({1}).".format(
                                        state_machine_dict['used_rafcon_version'], rafcon.__version__)
        note_about_possible_incompatibility = "The state machine will be loaded with no guarantee of success."

        if active_rafcon_version[0] > previously_used_rafcon_version[0]:
            # this is the default case
            # for a list of breaking changes please see: doc/breaking_changes.rst
            # logger.warning(rafcon_newer_than_sm_version)
            # logger.warning(note_about_possible_incompatibility)
            pass
        if active_rafcon_version[0] == previously_used_rafcon_version[0]:
            if active_rafcon_version[1] > previously_used_rafcon_version[1]:
                # this is the default case
                # for a list of breaking changes please see: doc/breaking_changes.rst
                # logger.info(rafcon_newer_than_sm_version)
                # logger.info(note_about_possible_incompatibility)
                pass
            elif active_rafcon_version[1] == previously_used_rafcon_version[1]:
                # Major and minor version of RAFCON and the state machine match
                # It should be safe to load the state machine, as the patch level does not change the format
                pass
            else:
                logger.warning(rafcon_older_than_sm_version)
                logger.warning(note_about_possible_incompatibility)
        else:
            logger.warning(rafcon_older_than_sm_version)
            logger.warning(note_about_possible_incompatibility)

    state_machine = StateMachine.from_dict(state_machine_dict, state_machine_id)
    if "root_state_storage_id" not in state_machine_dict:
        root_state_storage_id = state_machine_dict['root_state_id']
        state_machine.supports_saving_state_names = False
    else:
        root_state_storage_id = state_machine_dict['root_state_storage_id']

    root_state_path = os.path.join(base_path, root_state_storage_id)
    state_machine.file_system_path = base_path
    dirty_states = []
    state_machine.root_state = load_state_recursively(parent=state_machine, state_path=root_state_path,
                                                      dirty_states=dirty_states)
    if state_machine.root_state is None:
        return  # a corresponding exception has been handled with a proper error log in load_state_recursively
    if len(dirty_states) > 0:
        state_machine.marked_dirty = True
    else:
        state_machine.marked_dirty = False

    hierarchy_level = 0
    number_of_states, hierarchy_level = state_machine.root_state.get_states_statistics(hierarchy_level)
    logger.debug("Loaded state machine ({1}) has {0} states. (Max hierarchy level {2})".format(
        number_of_states, base_path, hierarchy_level))
    logger.debug("Loaded state machine ({1}) has {0} transitions.".format(
        state_machine.root_state.get_number_of_transitions(), base_path))
    logger.debug("Loaded state machine ({1}) has {0} data flows.".format(
        state_machine.root_state.get_number_of_data_flows(), base_path))

    return state_machine


def load_state_from_path(state_path):
    """Loads a state from a given path

    :param state_path: The path of the state on the file system.
    :return: the loaded state
    """
    return load_state_recursively(parent=None, state_path=state_path)


def load_state_recursively(parent, state_path=None, dirty_states=[]):
    """Recursively loads the state

    It calls this method on each sub-state of a container state.

    :param parent:  the root state of the last load call to which the loaded state will be added
    :param state_path: the path on the filesystem where to find the meta file for the state
    :param dirty_states: a dict of states which changed during loading
    :return:
    """
    from rafcon.core.states.execution_state import ExecutionState
    from rafcon.core.states.container_state import ContainerState
    from rafcon.core.states.hierarchy_state import HierarchyState

    path_core_data = os.path.join(state_path, FILE_NAME_CORE_DATA)

    logger.debug("Load state recursively: {0}".format(str(state_path)))

    # TODO: Should be removed with next minor release
    if not os.path.exists(path_core_data):
        path_core_data = os.path.join(state_path, FILE_NAME_CORE_DATA_OLD)

    try:
        state_info = load_data_file(path_core_data)
    except ValueError as e:
        logger.exception("Error while loading state data: {0}".format(e))
        return
    except LibraryNotFoundException as e:
        logger.error("Library could not be loaded: {0}\n"
                     "Skipping library and continuing loading the state machine".format(e))
        state_info = storage_utils.load_objects_from_json(path_core_data, as_dict=True)
        state_id = state_info["state_id"]
        dummy_state = HierarchyState(LIBRARY_NOT_FOUND_DUMMY_STATE_NAME, state_id=state_id)
        # set parent of dummy state
        if isinstance(parent, ContainerState):
            parent.add_state(dummy_state, storage_load=True)
        else:
            dummy_state.parent = parent
        return dummy_state

    # Transitions and data flows are not added when loading a state, as also states are not added.
    # We have to wait until the child states are loaded, before adding transitions and data flows, as otherwise the
    # validity checks for transitions and data flows would fail
    if not isinstance(state_info, tuple):
        state = state_info
    else:
        state = state_info[0]
        transitions = state_info[1]
        data_flows = state_info[2]

    # set parent of state
    if parent is not None and isinstance(parent, ContainerState):
        parent.add_state(state, storage_load=True)
    else:
        state.parent = parent

    # read script file if state is an ExecutionState
    if isinstance(state, ExecutionState):
        script_text = read_file(state_path, state.script.filename)
        state.script.set_script_without_compilation(script_text)

    # load semantic data
    try:
        semantic_data = load_data_file(os.path.join(state_path, SEMANTIC_DATA_FILE))
        state.semantic_data = semantic_data
    except Exception as e:
        # semantic data file does not have to be there
        pass

    one_of_my_child_states_not_found = False

    # load child states
    for p in os.listdir(state_path):
        child_state_path = os.path.join(state_path, p)
        if os.path.isdir(child_state_path):
            if not os.path.exists(os.path.join(child_state_path, FILE_NAME_CORE_DATA)):
                # this means that child_state_path is a folder, not containing a valid state
                # this also happens when pip creates __pycache__ folders for the script.py files upon installing rafcon
                continue
            child_state = load_state_recursively(state, child_state_path, dirty_states)
            if not child_state:
                return None
            if child_state.name is LIBRARY_NOT_FOUND_DUMMY_STATE_NAME:
                one_of_my_child_states_not_found = True

    if one_of_my_child_states_not_found:
        # omit adding transitions and data flows in this case
        pass
    else:
        # Now we can add transitions and data flows, as all child states were added
        if isinstance(state_info, tuple):
            # safe version
            # state.transitions = transitions
            # state.data_flows = data_flows
            state._transitions = transitions
            for _, transition in state.transitions.items():
                transition._parent = ref(state)
            state._data_flows = data_flows
            for _, data_flow in state.data_flows.items():
                data_flow._parent = ref(state)

    state.file_system_path = state_path

    if state.marked_dirty:
        dirty_states.append(state)

    return state


def load_data_file(path_of_file):
    """ Loads the content of a file by using json.load.

    :param path_of_file: the path of the file to load
    :return: the file content as a string
    :raises exceptions.ValueError: if the file was not found
    """
    if os.path.exists(path_of_file):
        return storage_utils.load_objects_from_json(path_of_file)
    raise ValueError("Data file not found: {0}".format(path_of_file))


def limit_text_max_length(text, max_length, separator='_'):
    """
    Limits the length of a string. The returned string will be the first `max_length/2` characters of the input string
    plus a separator plus the last `max_length/2` characters of the input string.
    
    :param text: the text to be limited
    :param max_length: the maximum length of the output string
    :param separator: the separator between the first "max_length"/2 characters of the input string and
                      the last "max_length/2" characters of the input string
    :return: the shortened input string
    """
    if max_length is not None:
        if isinstance(text, string_types) and len(text) > max_length:
            max_length = int(max_length)
            half_length = float(max_length - 1) / 2
            return text[:int(math.ceil(half_length))] + separator + text[-int(math.floor(half_length)):]
    return text


def clean_path_element(text, max_length=None, separator='_'):
    """ Replace characters that conflict with a free OS choice when in a file system path.

    :param text: the string to be cleaned
    :param max_length: the maximum length of the output string
    :param separator: the separator used for rafcon.core.storage.storage.limit_text_max_length
    :return:
    """
    elements_to_replace = REPLACED_CHARACTERS_FOR_NO_OS_LIMITATION
    for elem, replace_with in elements_to_replace.items():
        text = text.replace(elem, replace_with)
    if max_length is not None:
        text = limit_text_max_length(text, max_length, separator)
    return text


def limit_text_to_be_path_element(text, max_length=None, separator='_'):
    """ Replace characters that are not in the valid character set of RAFCON.

    :param text: the string to be cleaned
    :param max_length: the maximum length of the output string
    :param separator: the separator used for rafcon.core.storage.storage.limit_text_max_length
    :return:
    """
    # TODO: Should there not only be one method i.e. either this one or "clean_path_element"
    elements_to_replace = {' ': '_', '*': '_'}
    for elem, replace_with in elements_to_replace.items():
        text = text.replace(elem, replace_with)
    text = re.sub('[^a-zA-Z0-9-_]', '', text)
    if max_length is not None:
        text = limit_text_max_length(text, max_length, separator)
    return text


def get_storage_id_for_state(state):
    """ Calculates the storage id of a state. This ID can be used for generating the file path for a state.

    :param rafcon.core.states.state.State state: state the storage_id should is composed for
    """
    if global_config.get_config_value('STORAGE_PATH_WITH_STATE_NAME'):
        max_length = global_config.get_config_value('MAX_LENGTH_FOR_STATE_NAME_IN_STORAGE_PATH')

        max_length_of_state_name_in_folder_name = 255 - len(ID_NAME_DELIMITER + state.state_id)
        # TODO: should we allow "None" in config file?
        if max_length is None or max_length == "None" or max_length > max_length_of_state_name_in_folder_name:
            if max_length_of_state_name_in_folder_name < len(state.name):
                logger.info("The storage folder name is forced to be maximal 255 characters in length.")
            max_length = max_length_of_state_name_in_folder_name

        return limit_text_to_be_path_element(state.name, max_length) + ID_NAME_DELIMITER + state.state_id
    else:
        return state.state_id
