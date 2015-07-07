"""
.. module:: library_manager
   :platform: Unix, Windows
   :synopsis: A module to handle all libraries for a statemachine

.. moduleauthor:: Sebastian Brunner


"""


from gtkmvc import Observable
import os

from awesome_tool.utils import log
logger = log.get_logger(__name__)
from awesome_tool.statemachine.storage.storage import StateMachineStorage
import awesome_tool.statemachine.config as config
from awesome_tool.statemachine import interface
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
            lib_path = os.path.expanduser(lib_path)
            if os.path.exists(lib_path):
                lib_path = os.path.realpath(lib_path)
                logger.debug('Adding libraries from {0}'.format(lib_path))
                self._library_paths[lib_key] = lib_path
                self._libraries[lib_key] = {}
                self.add_libraries_from_path(lib_path, self._libraries[lib_key])
                if not sys.version_info < (2, 7):
                    self._libraries[lib_key] = OrderedDict(sorted(self._libraries[lib_key].items()))
                else:
                    self._libraries[lib_key] = dict(sorted(self._libraries[lib_key].items()))
            else:
                logger.warn("Wrong path in config for LibraryManager: Path %s does not exist", lib_path)
        if not sys.version_info < (2, 7):
            self._libraries = OrderedDict(sorted(self._libraries.items()))
        else:
            self._libraries = dict(sorted(self._libraries.items()))
        logger.debug("Initialization of LibraryManager done.")

    def add_libraries_from_path(self, lib_path, target_dict):
        """
        Adds all libraries specified in a given path and stores them into the provided library dictionary. The library
        entries in the dictionary consist only of the path to the library in the file system.
        :param lib_path: the path to add all libraries from
        :param target_dict: the target dictionary to store all loaded libraries to
        :return:
        """
        for lib in os.listdir(lib_path):
            if os.path.isdir(os.path.join(lib_path, lib)):
                if os.path.exists(os.path.join(os.path.join(lib_path, lib), StateMachineStorage.STATEMACHINE_FILE)):
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

        path_list = library_path.split("/")
        target_lib_dict = self.libraries

        while True:  # until the library is found or the user aborts

            if target_lib_dict is None:  # This cannot happen in the first iteration
                notice = "Cannot find library '{0}' in subfolder '{1}'. Please check your library path configuration." \
                            " If your library path is correct and the library was moved, please select the new root " \
                            "folder of the library. If not, please abort.".format(library_name, library_path)
                interface.show_notice_func(notice)
                new_library_path = interface.open_folder_func("Select root folder for library '{0}'".format(
                    library_name))
                if new_library_path is None:
                    # Cancel library search
                    raise AttributeError("Library '{0}' not found in subfolder {1}".format(library_name,
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
        return path, library_path, library_name
