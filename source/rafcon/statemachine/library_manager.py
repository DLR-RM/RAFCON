"""
.. module:: library_manager
   :platform: Unix, Windows
   :synopsis: A module to handle all libraries for a statemachine

.. moduleauthor:: Sebastian Brunner


"""

from gtkmvc import Observable
import os

from rafcon.utils import log
logger = log.get_logger(__name__)
from rafcon.statemachine.storage.storage import StateMachineStorage
from rafcon.statemachine.custom_exceptions import LibraryNotFoundException
import rafcon.statemachine.config as config
from rafcon.statemachine import interface
import sys
if not sys.version_info < (2, 7):
    from collections import OrderedDict


class LibraryManager(Observable):

    """This class manages all libraries specified in config.py.
    Libraries are essentially just (reusable) state machines.

    :ivar _libraries: a dictionary to hold  all libraries
    :ivar storage: the storage object to be able to load and save states
    """
    def __init__(self):
        Observable.__init__(self)
        self._libraries = {}
        self._library_paths = {}
        logger.debug("Initializing Storage object ...")
        self.storage = StateMachineStorage("../")
        # a list to hold all library state already manually replaced by the user
        self._replaced_libraries = {}
        # a list to hold all library states that were skipped by the user during the replacement procedure
        self._skipped_states = {}

    def initialize(self):
        """
        Initializes the library manager. It searches through all library paths given in the config file for
        libraries, and loads the states.

        This cannot be done in the __init__ function as the library_manager can be compiled and executed by
        singleton.py before the state*.pys are loaded
        :return:
        """
        logger.debug("Initializing LibraryManager: Loading libraries ... ")
        self._libraries = {}
        self._library_paths = {}
        for lib_key, lib_path in config.global_config.get_config_value("LIBRARY_PATHS").iteritems():
            # Replace ~ with /home/user
            lib_path = os.path.expanduser(lib_path)
            # Replace environment variables
            lib_path = os.path.expandvars(lib_path)
            # If the path is relative, assume it is relative to the config file directory
            if lib_path.startswith('.'):
                lib_path = os.path.join(config.global_config.path, lib_path)
            # Clean path, e.g. replace /./ with /
            lib_path = os.path.abspath(lib_path)

            if os.path.exists(lib_path):
                lib_path = os.path.realpath(lib_path)
                logger.debug("Adding library '{1}' from {0}".format(lib_path, lib_key))
                self._library_paths[lib_key] = lib_path
                self._libraries[lib_key] = {}
                self.add_libraries_from_path(lib_path, self._libraries[lib_key])
                if not sys.version_info < (2, 7):
                    self._libraries[lib_key] = OrderedDict(sorted(self._libraries[lib_key].items()))
                else:
                    self._libraries[lib_key] = dict(sorted(self._libraries[lib_key].items()))
            else:
                logger.warn("Wrong path in config for library with name: '{0}' because path {1} does not exist"
                            "".format(lib_key, lib_path))
        if not sys.version_info < (2, 7):
            self._libraries = OrderedDict(sorted(self._libraries.items()))
        else:
            self._libraries = dict(sorted(self._libraries.items()))
        logger.debug("Initialization of LibraryManager done")

    def add_libraries_from_path(self, lib_path, target_dict):
        """
        Adds all libraries specified in a given path and stores them into the provided library dictionary. The library
        entries in the dictionary consist only of the path to the library in the file system.
        :param lib_path: the path to add all libraries from
        :param target_dict: the target dictionary to store all loaded libraries to
        :return:
        """
        for lib in os.listdir(lib_path):
            if os.path.isdir(os.path.join(lib_path, lib)) and not '.' == lib[0]:
                if os.path.exists(os.path.join(os.path.join(lib_path, lib), StateMachineStorage.STATEMACHINE_FILE)) \
                        or os.path.exists(os.path.join(os.path.join(lib_path, lib),
                                                       StateMachineStorage.STATEMACHINE_FILE_OLD)):
                    self.add_library(lib, lib_path, target_dict)
                else:
                    target_dict[lib] = {}
                    self.add_libraries_from_path(os.path.join(lib_path, lib), target_dict[lib])
                    if not sys.version_info < (2, 7):
                        target_dict[lib] = OrderedDict(sorted(target_dict[lib].items()))
                    else:
                        target_dict[lib] = dict(sorted(target_dict[lib].items()))

    def add_library(self, lib, lib_path, target_dict):
        """
        Adds a single library path to the specified library dictionary.
        :param lib: the library to add
        :param lib_path: the path to the library specified in lib
        :param target_dict: the library dictionary to save the loaded library to
        :return:
        """
        self.storage.base_path = lib_path
        target_dict[lib] = os.path.join(lib_path, lib)

    @Observable.observed
    def refresh_libraries(self):
        """
        Deletes all loaded libraries and reloads them from the file system.
        :return:
        """
        self.initialize()

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def libraries(self):
        """Property for the _libraries field

        """
        return self._libraries

    @libraries.setter
    @Observable.observed
    def libraries(self, libraries):
        if not isinstance(libraries, dict):
            raise TypeError("libraries must be of type dict")

        self._libraries = libraries

    def get_os_path_to_library(self, library_path, library_name):
        """
        This function retrieves the file system path of a library specified by a path and a name in the case that the
        library does not exist any more at its original location. The user has to specify an alternative location.
        :param library_path:  the path of the library, that must be relative to a library path given in the config.yaml
        :param library_name: the name of the library
        :return:
        """
        path_list = library_path.split("/")
        target_lib_dict = self.libraries

        original_path_and_name = library_path+library_name

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
                             "with file system path \"{2}\" and library path \"{3}\"".format(str(library_path),
                                                                                     str(library_name),
                                                                                     str(new_path),
                                                                                     str(new_library_path)))
            print "RETURN", new_path, new_library_path, library_name
            return new_path, new_library_path, library_name

        # a boolean to indicate if a state was regularly found or by the help of the user
        regularly_found = True

        while True:  # until the library is found or the user aborts

            if target_lib_dict is None:  # This cannot happen in the first iteration
                regularly_found = False
                notice = "Cannot find library '{0}' in subfolder '{1}'. Please check your library path configuration." \
                            " If your library path is correct and the library was moved, please select the new root " \
                            "folder of the library. If not, please abort.".format(library_name, library_path)
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

                path_list = rel_path.split('/')
                library_name = path_list[-1]
                path_list = path_list[:-1]
                path_list.insert(0, library_key)
                library_path = '/'.join(path_list)
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
        import copy
        self._replaced_libraries[original_path_and_name] = (path, library_path, copy.copy(regularly_found))
        print "return end", path, library_path, library_name
        return path, library_path, library_name
