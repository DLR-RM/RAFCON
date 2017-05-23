# Copyright (C) 2015-2017 DLR
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
from gtkmvc import Observable

import rafcon
from rafcon.core import interface
from rafcon.core.storage import storage
from rafcon.core.custom_exceptions import LibraryNotFoundException
import rafcon.core.config as config

from rafcon.utils import log
logger = log.get_logger(__name__)

try:
    from collections import OrderedDict
except ImportError:
    OrderedDict = dict


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
        self._library_paths = {}
        logger.debug("Initializing Storage object ...")
        # a list to hold all library state already manually replaced by the user
        self._replaced_libraries = {}
        # a list to hold all library states that were skipped by the user during the replacement procedure
        self._skipped_states = []
        self._skipped_library_roots = []

    def initialize(self):
        """Initializes the library manager

        It searches through all library paths given in the config file for libraries, and loads the states.

        This cannot be done in the __init__ function as the library_manager can be compiled and executed by
        singleton.py before the state*.pys are loaded
        """
        logger.debug("Initializing LibraryManager: Loading libraries ... ")
        self._libraries = {}
        self._library_paths = {}
        self._replaced_libraries = {}
        self._skipped_states = []
        self._skipped_library_roots = []

        # 1. Load libraries from config.yaml
        for library_root_key, library_root_path in config.global_config.get_config_value("LIBRARY_PATHS").iteritems():
            library_root_path = self._clean_path(library_root_path)
            if os.path.exists(library_root_path):
                logger.debug("Adding library '{1}' from {0}".format(library_root_path, library_root_key))
                self._load_library_from_root_path(library_root_key, library_root_path)
            else:
                logger.warn("Configured path for library '{}' does not exist: {}".format(library_root_key,
                                                                                         library_root_path))

        # 2. Load libraries from RAFCON_LIBRARY_PATH
        library_path_env = os.environ.get('RAFCON_LIBRARY_PATH', '')
        library_paths = set(library_path_env.split(os.pathsep))
        for library_root_path in library_paths:
            if not library_root_path:
                continue
            library_root_path = self._clean_path(library_root_path)
            if not os.path.exists(library_root_path):
                logger.warn("The library specified in RAFCON_LIBRARY_PATH does not exist: {}".format(library_root_path))
                continue
            _, library_root_key = os.path.split(library_root_path)
            if library_root_key in self._libraries:
                logger.warn("The library '{}' is already existing and will be overridden with '{}'".format(
                    library_root_key, library_root_path))
            self._load_library_from_root_path(library_root_key, library_root_path)
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

    def _load_library_from_root_path(self, library_root_key, library_root_path):
        self._library_paths[library_root_key] = library_root_path
        self._libraries[library_root_key] = {}
        self._load_nested_libraries(library_root_path, self._libraries[library_root_key])
        self._libraries[library_root_key] = OrderedDict(sorted(self._libraries[library_root_key].items()))

    @classmethod
    def _load_nested_libraries(cls, library_path, target_dict):
        """Recursively load libraries within path

        Adds all libraries specified in a given path and stores them into the provided library dictionary. The library
        entries in the dictionary consist only of the path to the library in the file system.

        :param library_path: the path to add all libraries from
        :param target_dict: the target dictionary to store all loaded libraries to
        """
        for library_name in os.listdir(library_path):
            full_library_path = os.path.join(library_path, library_name)
            if os.path.isdir(full_library_path) and library_name[0] != '.':
                if os.path.exists(os.path.join(full_library_path, storage.STATEMACHINE_FILE)) \
                        or os.path.exists(os.path.join(full_library_path, storage.STATEMACHINE_FILE_OLD)):
                    target_dict[library_name] = full_library_path
                else:
                    target_dict[library_name] = {}
                    cls._load_nested_libraries(full_library_path, target_dict[library_name])
                    target_dict[library_name] = OrderedDict(sorted(target_dict[library_name].items()))

    @Observable.observed
    def refresh_libraries(self):
        """Deletes all loaded libraries and reloads them from the file system
        """
        self.initialize()

    #########################################################################
    # Properties for all class fields that must be observed by gtkmvc
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
    def library_paths(self):
        """Getter for library paths
        """
        return self._library_paths

    def get_os_path_to_library(self, library_path, library_name, allow_user_interaction=True):
        """Find library_os_path of library

        This function retrieves the file system library_os_path of a library specified by a library_os_path and a name. In case the library
        does not exist any more at its original location, the user has to specify an alternative location.

        :param library_path: the library_path of the library, that must be relative and within a library_root_path 
                             given in the config.yaml by LIBRARY_PATHS
        :param library_name: the name of the library
        :param allow_user_interaction: Whether the user may be asked to specify library location
        :return: library library_os_path within filesystem, library_os_path within library, library name
        :rtype: str, str, str
        :raises rafcon.core.custom_exceptions.LibraryNotFoundException: if the cannot be found
        """
        # TODO this method needs refactoring it is to cryptic (e.g. end of while loop) and similar code is
        # TODO get_library_path_and_name_for_os_path
        path_list = library_path.split(os.sep)

        original_path_and_name = library_path + library_name
        library_path_root = library_path.split(os.sep)[0]

        if path_list and (library_path.startswith(os.sep) or library_path.endswith(os.sep)):
            raise LibraryNotFoundException("A library library_os_path is not considered to start or end with {2} like "
                                           "'{0}' with library name {1}".format(library_path, library_name, os.sep))

        if not self._library_paths:
            raise LibraryNotFoundException("There are no libraries registered")

        # skip already skipped states
        if original_path_and_name in self._skipped_states or library_path_root in self._skipped_library_roots:
            # if an already skipped state shall be loaded again, directly raise the exception to jump over this state
            raise LibraryNotFoundException("Library '{0}' not found in subfolder {1}".format(library_name,
                                                                                             library_path))

        # replace already replaced states
        if original_path_and_name in self._replaced_libraries:
            new_path = self._replaced_libraries[original_path_and_name][0]
            new_library_sys_path = self._replaced_libraries[original_path_and_name][1]

            # only show debug message if a state is automatically replaced by the appropriate library state
            # chosen by the user before
            if not self._replaced_libraries[original_path_and_name][2]:
                logger.debug("The library with library library_os_path \"{0}\" and name \"{1}\" "
                             "is automatically replaced by the library "
                             "with file system library_os_path \"{2}\" and library library_os_path \"{3}\"".format(library_path,
                                                                                             library_name,
                                                                                             new_path,
                                                                                             new_library_sys_path))
            return new_path, new_library_sys_path, library_name

        # a boolean to indicate if a state was regularly found or by the help of the user
        regularly_found = True

        target_lib_dict = self._get_target_lib_dict_leaf(path_list)
        library_os_path = None if target_lib_dict is None else target_lib_dict[library_name]
        while library_os_path is None:  # until the library is found or the user aborts

            regularly_found = False
            new_library_sys_path = None
            if allow_user_interaction:
                notice = "Cannot find library '{0}' in subfolder '{1}'. Please check your library library_os_path " \
                         "configuration. If your library library_os_path is correct and the library was moved, please select" \
                         " the new root folder of the library. If not, please abort.".format(library_name,
                                                                                             library_path)
                interface.show_notice_func(notice)
                new_library_sys_path = interface.open_folder_func("Select root folder for library '{0}'".format(
                    library_name))
            if new_library_sys_path is None:
                # User clicked cancel => cancel library search
                # If the library root is existent (e.g. "generic") and only the specific library state is not (
                # e.g. "generic/wait", then the state is added to the skipped states.
                # If the library root is not existing, we ignore the whole library, preventing the user from
                # endless dialogs for each missing library state.
                if library_path_root not in self.libraries:
                    self._skipped_library_roots.append(library_path_root)
                else:
                    self._skipped_states.append(original_path_and_name)
                raise LibraryNotFoundException("Library '{0}' not found in subfolder {1}".format(library_name,
                                                                                                 library_path))

            if not os.path.exists(new_library_sys_path):
                logger.error('Specified library_os_path does not exist')
                continue

            library_root_key = self._get_library_root_key_for_abs_path(new_library_sys_path)
            if library_root_key is None:
                logger.error("Specified library_os_path not within library library_os_path list or your config file")
                continue  # Allow the user to change the directory

            # construct library_name and library_path from library_root_key and new_library_sys_path
            rel_path = os.path.relpath(new_library_sys_path, self._library_paths[library_root_key])
            path_list = rel_path.split(os.sep)
            library_name = path_list[-1]
            path_list = [library_root_key] + path_list[:-1]
            library_path = os.path.join(*path_list)

            # verification
            target_lib_dict = self._get_target_lib_dict_leaf(path_list)
            library_os_path = None if target_lib_dict is None else target_lib_dict[library_name]
            if library_os_path is not None:
                assert library_os_path == new_library_sys_path

        # save the replacement in order that for a future occurrence the correct library_os_path can be used,
        # without asking the user for the correct library_os_path
        self._replaced_libraries[original_path_and_name] = (library_os_path, library_path, regularly_found)
        return library_os_path, library_path, library_name

    def _get_target_lib_dict_leaf(self, path_list):
        target_lib_dict = self.libraries
        # go down the path to the correct library
        for path_element in path_list:
            if path_element not in target_lib_dict:  # Library cannot be found
                target_lib_dict = None
                break
            target_lib_dict = target_lib_dict[path_element]
        return target_lib_dict

    def _get_library_root_key_for_abs_path(self, path):
        path = os.path.realpath(path)
        library_root_key = None
        for library_root_key, library_root_path in self._library_paths.iteritems():
            rel_path = os.path.relpath(path, library_root_path)
            if rel_path.startswith('..'):
                library_root_key = None
                continue
            else:
                break
        return library_root_key

    def is_path_within_library_path_list(self, path):
        return True if self._get_library_root_key_for_abs_path(path) is not None else False

    def is_library_in_libraries(self, library_path, library_name):
        library_path_elements = library_path.split(os.sep)
        sub_library = self.libraries
        for path_element in library_path_elements:
            if path_element in sub_library:
                sub_library = sub_library[path_element]
            else:
                return False
        if library_name in sub_library:
            return True
        else:
            return False

    def is_os_path_in_library_paths(self, os_path):
        return any([root_path == os_path[:len(root_path)] for root_path in self._library_paths.values()])

    def get_library_path_and_name_for_os_path(self, path):
        library_path = None
        library_name = None
        if self.is_os_path_in_library_paths(path):
            for key, root_path in self._library_paths.iteritems():
                if len(root_path) <= len(path) and root_path == path[:len(root_path)]:
                    library_root_path = root_path
                    library_root_name = key
                    path_elements_without_library_root = path[len(library_root_path)+1:].split(os.sep)
                    library_name = path_elements_without_library_root[-1]
                    sub_library_path = ''
                    if len(path_elements_without_library_root[:-1]):
                        sub_library_path = os.sep + os.sep.join(path_elements_without_library_root[:-1])
                    library_path = library_root_name + sub_library_path
                    break
        return library_path, library_name

    def get_library_instance(self, library_path, library_name):
        if self.is_library_in_libraries(library_path, library_name):
            from rafcon.core.states.library_state import LibraryState
            return LibraryState(library_path, library_name, "0.1")
        else:
            logger.warning("Library manager will not create a library instance which is not in the mounted libraries.")

    def remove_library_from_file_system(self, library_path, library_name):
        library_file_system_path = self.get_os_path_to_library(library_path, library_name)[0]
        shutil.rmtree(library_file_system_path)
        self.refresh_libraries()
