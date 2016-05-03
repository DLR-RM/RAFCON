"""
.. module:: library_manager
   :platform: Unix, Windows
   :synopsis: A module to handle all libraries for a statemachine

.. moduleauthor:: Sebastian Brunner
"""

import os
import sys
from gtkmvc import Observable

from rafcon.statemachine import interface
from rafcon.statemachine.storage import storage
from rafcon.statemachine.custom_exceptions import LibraryNotFoundException
import rafcon.statemachine.config as config

from rafcon.utils import log
logger = log.get_logger(__name__)

if not sys.version_info < (2, 7):
    from collections import OrderedDict as ordered_dict
else:
    ordered_dict = dict


class LibraryManager(Observable):
    """This class manages all libraries

    Libraries are essentially just (reusable) state machines.

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
        self._skipped_states = {}

    def initialize(self):
        """Initializes the library manager

        It searches through all library paths given in the config file for libraries, and loads the states.

        This cannot be done in the __init__ function as the library_manager can be compiled and executed by
        singleton.py before the state*.pys are loaded
        """
        logger.debug("Initializing LibraryManager: Loading libraries ... ")
        self._libraries = {}
        self._library_paths = {}

        # 1. Load libraries from config.yaml
        for library_key, library_path in config.global_config.get_config_value("LIBRARY_PATHS").iteritems():
            library_path = self._clean_path(library_path)
            if os.path.exists(library_path):
                logger.debug("Adding library '{1}' from {0}".format(library_path, library_key))
                self._load_library_from_root_path(library_key, library_path)
            else:
                logger.warn("Configured path for library '{}' does not exist: {}".format(library_key, library_path))

        # 2. Load libraries from RAFCON_LIBRARY_PATH
        library_path_env = os.environ.get('RAFCON_LIBRARY_PATH', '')
        library_paths = set(library_path_env.split(os.pathsep))
        for library_path in library_paths:
            if not library_path:
                continue
            library_path = self._clean_path(library_path)
            if not os.path.exists(library_path):
                logger.warn("The library specified in RAFCON_LIBRARY_PATH does not exist: {}".format(library_path))
                continue
            _, library_key = os.path.split(library_path)
            if library_key in self._libraries:
                logger.warn("The library '{}' is already existing and will be overridden with '{}'".format(
                    library_key, library_path))
            self._load_library_from_root_path(library_key, library_path)
            logger.debug("Adding library '{1}' from {0}".format(library_path, library_key))

        self._libraries = ordered_dict(sorted(self._libraries.items()))
        logger.debug("Initialization of LibraryManager done")

    @staticmethod
    def _clean_path(path):
        path = path.replace('"', '')
        path = path.replace("'", '')
        # Replace ~ with /home/user
        path = os.path.expanduser(path)
        # Replace environment variables
        path = os.path.expandvars(path)
        # If the path is relative, assume it is relative to the config file directory
        if path.startswith('.'):
            path = os.path.join(config.global_config.path, path)
        # Clean path, e.g. replace /./ with /
        path = os.path.abspath(path)
        # Eliminate symbolic links
        path = os.path.realpath(path)
        return path

    def _load_library_from_root_path(self, library_key, library_path):
        self._library_paths[library_key] = library_path
        self._libraries[library_key] = {}
        self._load_nested_libraries(library_path, self._libraries[library_key])
        self._libraries[library_key] = ordered_dict(sorted(self._libraries[library_key].items()))

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
                    target_dict[library_name] = ordered_dict(sorted(target_dict[library_name].items()))

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

    def get_os_path_to_library(self, library_path, library_name, allow_user_interaction=True):
        """Find path of library

        This function retrieves the file system path of a library specified by a path and a name. In case the library
        does not exist any more at its original location, the user has to specify an alternative location.

        :param library_path:  the path of the library, that must be relative to a library path given in the config.yaml
        :param library_name: the name of the library
        :param allow_user_interaction: Whether the user may be asked to specify library location
        :return: library path within filesystem, path within library, library name
        :rtype: str, str, str
        """
        path_list = library_path.split(os.sep)
        target_lib_dict = self.libraries

        original_path_and_name = library_path + library_name

        if not self._library_paths:
            raise LibraryNotFoundException("There are no libraries registered")

        # skip already skipped states
        if original_path_and_name in self._skipped_states:
            # if an already skipped state shall be loaded again, directly raise the exception to jump over this state
            raise LibraryNotFoundException("Library '{0}' not found in subfolder {1}".format(library_name,
                                                                                             library_path))

        # replace already replaced states
        if original_path_and_name in self._replaced_libraries:
            new_path = self._replaced_libraries[original_path_and_name][0]
            new_library_path = self._replaced_libraries[original_path_and_name][1]

            # only show debug message if a state is automatically replaced by the appropriate library state
            # chosen by the user before
            if not self._replaced_libraries[original_path_and_name][2]:
                logger.debug("The library with library path \"{0}\" and name \"{1}\" "
                             "is automatically replaced by the library "
                             "with file system path \"{2}\" and library path \"{3}\"".format(library_path,
                                                                                             library_name,
                                                                                             new_path,
                                                                                             new_library_path))
            return new_path, new_library_path, library_name

        # a boolean to indicate if a state was regularly found or by the help of the user
        regularly_found = True

        while True:  # until the library is found or the user aborts

            if target_lib_dict is None:  # This cannot happen in the first iteration
                regularly_found = False
                new_library_path = None
                if allow_user_interaction:
                    notice = "Cannot find library '{0}' in subfolder '{1}'. Please check your library path " \
                             "configuration. If your library path is correct and the library was moved, please select" \
                             " the new root folder of the library. If not, please abort.".format(library_name,
                                                                                                 library_path)
                    interface.show_notice_func(notice)
                    new_library_path = interface.open_folder_func("Select root folder for library '{0}'".format(
                        library_name))
                if new_library_path is None:
                    # Cancel library search
                    self._skipped_states[original_path_and_name] = True
                    raise LibraryNotFoundException("Library '{0}' not found in subfolder {1}".format(library_name,
                                                                                                     library_path))

                if not os.path.exists(new_library_path):
                    logger.error('Specified path does not exist')
                    continue

                new_library_path = os.path.realpath(new_library_path)
                for library_key, library_path in self._library_paths.iteritems():
                    rel_path = os.path.relpath(new_library_path, library_path)
                    if rel_path.startswith('..'):
                        library_key = None
                        continue
                    else:
                        break
                if library_key is None:
                    logger.error("Specified path not within library path list or your config file")
                    continue  # Allow the user to change the directory

                path_list = rel_path.split(os.sep)
                library_name = path_list[-1]
                path_list = path_list[:-1]
                path_list.insert(0, library_key)
                library_path = os.path.join(*path_list)
                target_lib_dict = self.libraries

            # go down the path to the correct library
            for path_element in path_list:
                if path_element not in target_lib_dict:  # Library cannot be found
                    target_lib_dict = None
                    break
                target_lib_dict = target_lib_dict[path_element]

            if target_lib_dict is None or library_name not in target_lib_dict:
                target_lib_dict = None
            else:
                break

        path = target_lib_dict[library_name]
        # save the replacement in order that for a future occurrence the correct path can be used, without asking
        # the user for the correct path
        self._replaced_libraries[original_path_and_name] = (path, library_path, regularly_found)
        return path, library_path, library_name
