# Copyright (C) 2015-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: library_manager
   :synopsis: A module to handle all libraries for a state machine

"""

import os
import shutil
import copy
import warnings
from collections import OrderedDict
from gtkmvc3.observable import Observable

from rafcon.core import interface
from rafcon.core.storage import storage
from rafcon.core.custom_exceptions import LibraryNotFoundException
import rafcon.core.config as config

from rafcon.utils import log
logger = log.get_logger(__name__)


class LibraryManager(Observable):
    """This class manages all libraries

    Libraries are essentially just (reusable) state machines. A set of state machines from path library_root_path 
    are mounted at point/name library_root_key in the library_manager.
    With library_path and library_name the respective library state machine is found in the library manager.
    Hereby a library_root_key has to be the first element of the library_path.
    The library_root_paths are handed in the config.yaml in the LIBRARY_PATHS dictionary as pairs of library_root_key 
    and library_root_path.
    The library_root_path can be relative paths and could include environment variables.
    A library is pointed on by the file system path library_os_path which again partial consists of 
    library_root_path + library_path (partly) + library_name.
    :ivar _libraries: a dictionary to hold  all libraries
    """

    def __init__(self):
        Observable.__init__(self)
        self._libraries = {}
        self._library_root_paths = {}
        logger.debug("Initializing Storage object ...")
        # a list to hold all library state already manually replaced by the user
        self._replaced_libraries = {}
        # a list to hold all library states that were skipped by the user during the replacement procedure
        self._skipped_states = []
        self._skipped_library_roots = []

        # loaded libraries
        self._loaded_libraries = {}
        self._libraries_instances = {}

    def prepare_destruction(self):
        self.clean_loaded_libraries()

    def clean_loaded_libraries(self):
        self._loaded_libraries.clear()

    def initialize(self):
        """Initializes the library manager

        It searches through all library paths given in the config file for libraries, and loads the states.

        This cannot be done in the __init__ function as the library_manager can be compiled and executed by
        singleton.py before the state*.pys are loaded
        """
        logger.debug("Initializing LibraryManager: Loading libraries ... ")
        self._libraries = {}
        self._library_root_paths = {}
        self._replaced_libraries = {}
        self._skipped_states = []
        self._skipped_library_roots = []

        # 1. Load libraries from config.yaml
        for library_root_key, library_root_path in config.global_config.get_config_value("LIBRARY_PATHS").items():
            library_root_path = self._clean_path(library_root_path)
            if os.path.exists(library_root_path):
                logger.debug("Adding library root key '{0}' from path '{1}'".format(
                    library_root_key, library_root_path))
                self._load_libraries_from_root_path(library_root_key, library_root_path)
            else:
                logger.warning("Configured path for library root key '{}' does not exist: {}".format(
                    library_root_key, library_root_path))

        # 2. Load libraries from RAFCON_LIBRARY_PATH
        library_path_env = os.environ.get('RAFCON_LIBRARY_PATH', '')
        library_paths = set(library_path_env.split(os.pathsep))
        for library_root_path in library_paths:
            if not library_root_path:
                continue
            library_root_path = self._clean_path(library_root_path)
            if not os.path.exists(library_root_path):
                logger.warning("The library specified in RAFCON_LIBRARY_PATH does not exist: {}".format(library_root_path))
                continue
            _, library_root_key = os.path.split(library_root_path)
            if library_root_key in self._libraries:
                if self._library_root_paths[library_root_key] == library_root_path:
                    logger.info("The library root key '{}' and root path '{}' exists multiple times in your environment"
                                " and will be skipped.".format(library_root_key, library_root_path))
                else:
                    logger.warning("The library '{}' is already existing and will be overridden with '{}'".format(
                        library_root_key, library_root_path))
                    self._load_libraries_from_root_path(library_root_key, library_root_path)
            else:
                self._load_libraries_from_root_path(library_root_key, library_root_path)
            logger.debug("Adding library '{1}' from {0}".format(library_root_path, library_root_key))

        self._libraries = OrderedDict(sorted(self._libraries.items()))
        logger.debug("Initialization of LibraryManager done")

    @staticmethod
    def _clean_path(path):
        """Create a fully fissile absolute system path with no symbolic links and environment variables"""
        path = path.replace('"', '')
        path = path.replace("'", '')
        # Replace ~ with /home/user
        path = os.path.expanduser(path)
        # Replace environment variables
        path = os.path.expandvars(path)
        # If the path is relative, assume it is relative to the config file directory
        if not os.path.isabs(path):
            path = os.path.join(config.global_config.path, path)
        # Clean path, e.g. replace /./ with /
        path = os.path.abspath(path)
        # Eliminate symbolic links
        path = os.path.realpath(path)
        return path

    def _load_libraries_from_root_path(self, library_root_key, library_root_path):
        self._library_root_paths[library_root_key] = library_root_path
        self._libraries[library_root_key] = {}
        self._load_nested_libraries(library_root_path, self._libraries[library_root_key])
        self._libraries[library_root_key] = OrderedDict(sorted(self._libraries[library_root_key].items()))

    def check_clean_path_of_library(self, folder_path, folder_name):
        library_root_path = self._library_root_paths[self._get_library_root_key_for_os_path(folder_path)]
        full_path = os.path.join(folder_path, folder_name)[len(library_root_path) + 1:]
        library_path = folder_path[len(library_root_path):]
        if not storage.clean_path(library_path) == library_path or not storage.clean_path(folder_name) == folder_name:
            not_allowed_characters = "'" + "', '".join(storage.REPLACED_CHARACTERS_FOR_NO_OS_LIMITATION.keys()) + "'"
            warnings.warn("The library path {1} is deprecated, please avoid the usage of {0}. If you rename the "
                          "state machine, remember to also adapt the related state machines using this library."
                          "".format(not_allowed_characters, full_path), log.RAFCONDeprecationWarning)
        return folder_path, folder_name

    def _load_nested_libraries(self, library_path, target_dict):
        """Recursively load libraries within path

        Adds all libraries specified in a given path and stores them into the provided library dictionary. The library
        entries in the dictionary consist only of the path to the library in the file system.

        :param library_path: the path to add all libraries from
        :param target_dict: the target dictionary to store all loaded libraries to
        """
        for library_name in os.listdir(library_path):
            library_folder_path, library_name = self.check_clean_path_of_library(library_path, library_name)
            full_library_path = os.path.join(library_path, library_name)
            if os.path.isdir(full_library_path) and library_name[0] != '.':
                if os.path.exists(os.path.join(full_library_path, storage.STATEMACHINE_FILE)) \
                        or os.path.exists(os.path.join(full_library_path, storage.STATEMACHINE_FILE_OLD)):
                    target_dict[library_name] = full_library_path
                else:
                    target_dict[library_name] = {}
                    self._load_nested_libraries(full_library_path, target_dict[library_name])
                    target_dict[library_name] = OrderedDict(sorted(target_dict[library_name].items()))

    @Observable.observed
    def refresh_libraries(self):
        """Deletes all loaded libraries and reloads them from the file system
        """
        self.initialize()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc3
    #########################################################################

    @property
    def libraries(self):
        """Getter for library tree
        """
        return self._libraries

    @libraries.setter
    @Observable.observed
    def libraries(self, libraries):
        if not isinstance(libraries, dict):
            raise TypeError("libraries must be of type dict")

        self._libraries = libraries

    @property
    def library_root_paths(self):
        """Getter for library paths
        """
        return self._library_root_paths

    def get_os_path_to_library(self, library_path, library_name, allow_user_interaction=True):
        """Find library_os_path of library

        This function retrieves the file system library_os_path of a library specified by a library_path and a 
        library_name. In case the library does not exist any more at its original location, the user has to specify 
        an alternative location.

        :param str library_path: The library_path of the library, that must be relative and within a library_root_path 
                             given in the config.yaml by LIBRARY_PATHS
        :param str library_name: The name of the library
        :param bool allow_user_interaction: Whether the user may be asked to specify library location
        :return: library_os_path within filesystem, library_path, library_name
        :rtype: str, str, str
        :raises rafcon.core.custom_exceptions.LibraryNotFoundException: if the cannot be found
        """
        original_path_and_name = os.path.join(library_path, library_name)
        library_path_root = library_path.split(os.sep)[0]

        if library_path.split(os.sep) and \
                (library_path.startswith(os.sep) or library_path.endswith(os.sep) or os.sep + os.sep in library_path):
            raise LibraryNotFoundException("A library_path is not considered to start or end with {2} or to have two "
                                           "path separators {2}{2} in a row like '{0}' with library name {1}"
                                           "".format(library_path, library_name, os.sep))

        if not self._library_root_paths:
            raise LibraryNotFoundException("There are no libraries registered")

        # skip already skipped states
        if original_path_and_name in self._skipped_states or library_path_root in self._skipped_library_roots:
            # if an already skipped state shall be loaded again, directly raise the exception to jump over this state
            raise LibraryNotFoundException("Library '{0}' not found in subfolder {1}".format(library_name,
                                                                                             library_path))

        # replace already replaced states
        if original_path_and_name in self._replaced_libraries:
            new_library_os_path = self._replaced_libraries[original_path_and_name][0]
            new_library_path = self._replaced_libraries[original_path_and_name][1]

            # only show debug message if a state is automatically replaced by the appropriate library state
            # chosen by the user before
            if not self._replaced_libraries[original_path_and_name][2]:
                logger.debug("The library with library_path \"{0}\" and name \"{1}\" "
                             "is automatically replaced by the library "
                             "with file system library_os_path \"{2}\" and library_path \"{3}\""
                             "".format(library_path, library_name, new_library_os_path, new_library_path))
            return new_library_os_path, new_library_path, library_name

        # a boolean to indicate if a state was regularly found or by the help of the user
        regularly_found = True

        library_os_path = self._get_library_os_path_from_library_dict_tree(library_path, library_name)
        while library_os_path is None:  # until the library is found or the user aborts

            regularly_found = False
            new_library_os_path = None
            if allow_user_interaction:
                notice = "Cannot find library '{0}' in library_path '{1}' in any of the library root paths. " \
                         "Please check your library root paths configuration in config.yaml " \
                         "LIBRARY_PATHS and environment variable RAFCON_LIBRARY_PATH. " \
                         "If your library_path is correct and the library was moved, please " \
                         "select the new root/library_os_path folder of the library which should be situated within a "\
                         "loaded library_root_path. If not, please abort.".format(library_name, library_path)
                interface.show_notice_func(notice)
                new_library_os_path = interface.open_folder_func("Select root folder for library name '{0}'"
                                                                 "".format(original_path_and_name))
            if new_library_os_path is None:
                # User clicked cancel => cancel library search
                # If the library root path is existent (e.g. "generic") and only the specific library state is not (
                # e.g. "generic/wait", then the state is added to the skipped states.
                # If the library root path is not existing, we ignore the whole library, preventing the user from
                # endless dialogs for each missing library state.
                if library_path_root not in self.libraries:
                    self._skipped_library_roots.append(library_path_root)
                else:
                    self._skipped_states.append(original_path_and_name)
                raise LibraryNotFoundException("Library '{0}' not found in sub-folder {1}".format(library_name,
                                                                                                  library_path))

            if not os.path.exists(new_library_os_path):
                logger.error('Specified library_os_path does not exist')
                continue

            # check if valid library_path and library_name can be created
            library_path, library_name = self.get_library_path_and_name_for_os_path(new_library_os_path)
            if library_path is None:
                logger.error("Specified library_os_path not within loaded library_root_path list or your config.yaml "
                             "file LIBRARY_PATHS or in the list of paths in environment variable RAFCON_LIBRARY_PATH")
                continue  # Allow the user to change the directory

            # verification if library is also in library tree
            library_os_path = self._get_library_os_path_from_library_dict_tree(library_path, library_name)
            if library_os_path is not None:
                assert library_os_path == new_library_os_path

        # save the replacement in order that for a future occurrence the correct library_os_path can be used,
        # without asking the user for the correct library_os_path
        self._replaced_libraries[original_path_and_name] = (library_os_path, library_path, regularly_found)
        return library_os_path, library_path, library_name

    def _get_library_os_path_from_library_dict_tree(self, library_path, library_name):
        """Hand verified library os path from libraries dictionary tree."""
        if library_path is None or library_name is None:
            return None
        path_list = library_path.split(os.sep)
        target_lib_dict = self.libraries
        # go down the path to the correct library
        for path_element in path_list:
            if path_element not in target_lib_dict:  # Library cannot be found
                target_lib_dict = None
                break
            target_lib_dict = target_lib_dict[path_element]
        return None if target_lib_dict is None or library_name not in target_lib_dict else target_lib_dict[library_name]

    def _get_library_root_key_for_os_path(self, path):
        """Return library root key if path is within library root paths"""
        path = os.path.realpath(path)
        library_root_key = None
        for library_root_key, library_root_path in self._library_root_paths.items():
            rel_path = os.path.relpath(path, library_root_path)
            if rel_path.startswith('..'):
                library_root_key = None
                continue
            else:
                break
        return library_root_key

    def is_os_path_within_library_root_paths(self, path):
        return True if self._get_library_root_key_for_os_path(path) is not None else False

    def is_library_in_libraries(self, library_path, library_name):
        library_os_path = self._get_library_os_path_from_library_dict_tree(library_path, library_name)
        return True if library_os_path is not None else False

    def get_library_path_and_name_for_os_path(self, path):
        """Generate valid library_path and library_name

        The method checks if the given os path is in the list of loaded library root paths and use respective 
        library root key/mounting point to concatenate the respective library_path and separate respective library_name.

        :param str path: A library os path a library is situated in.
        :return: library path library name
        :rtype: str, str
        """
        path = os.path.realpath(path)
        library_path = None
        library_name = None
        library_root_key = self._get_library_root_key_for_os_path(path)
        if library_root_key is not None:
            library_root_path = self._library_root_paths[library_root_key]
            path_elements_without_library_root = path[len(library_root_path)+1:].split(os.sep)
            library_name = path_elements_without_library_root[-1]
            sub_library_path = ''
            if len(path_elements_without_library_root[:-1]):
                sub_library_path = os.sep + os.sep.join(path_elements_without_library_root[:-1])
            library_path = library_root_key + sub_library_path
        return library_path, library_name

    def get_library_instance(self, library_path, library_name):
        """Generate a Library instance from within libraries dictionary tree."""
        if self.is_library_in_libraries(library_path, library_name):
            from rafcon.core.states.library_state import LibraryState
            return LibraryState(library_path, library_name, "0.1")
        else:
            logger.warning("Library manager will not create a library instance which is not in the mounted libraries.")

    def get_library_state_copy_instance(self, lib_os_path):
        """ A method to get a state copy of the library specified via the lib_os_path.

        :param lib_os_path: the location of the library to get a copy for
        :return:
        """

        # originally libraries were called like this; DO NOT DELETE; interesting for performance tests
        # state_machine = storage.load_state_machine_from_path(lib_os_path)
        # return state_machine.version, state_machine.root_state

        # TODO observe changes on file system and update data
        if lib_os_path in self._loaded_libraries:
            # this list can also be taken to open library state machines TODO -> implement it -> because faster
            state_machine = self._loaded_libraries[lib_os_path]
            # logger.info("Take copy of {0}".format(lib_os_path))
            # as long as the a library state root state is never edited so the state first has to be copied here
            state_copy = copy.deepcopy(state_machine.root_state)
            return state_machine.version, state_copy
        else:
            state_machine = storage.load_state_machine_from_path(lib_os_path)
            self._loaded_libraries[lib_os_path] = state_machine
            if config.global_config.get_config_value("NO_PROGRAMMATIC_CHANGE_OF_LIBRARY_STATES_PERFORMED", False):
                return state_machine.version, state_machine.root_state
            else:
                state_copy = copy.deepcopy(state_machine.root_state)
                return state_machine.version, state_copy

    def remove_library_from_file_system(self, library_path, library_name):
        """Remove library from hard disk."""
        library_file_system_path = self.get_os_path_to_library(library_path, library_name)[0]
        shutil.rmtree(library_file_system_path)
        self.refresh_libraries()
