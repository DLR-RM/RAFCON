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
import config
from awesome_tool.statemachine.storage.storage import Storage


class LibraryManager(Observable):

    """This class manages all libraries specified in config.py.
    Libraries are essentially just (reusable) state machines.

    :ivar _libraries: a dictionary to hold  all libraries
    :ivar storage: the storage object to be able to load and save states
    """
    def __init__(self):
        Observable.__init__(self)
        self._libraries = {}
        logger.debug("Initializing Storage object ...")
        self.storage = Storage("../")

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
        for lib_key, lib_path in config.LIBRARY_PATHS.iteritems():
            if os.path.exists(lib_path):
                self._libraries[lib_key] = {}
                self.add_libraries_from_path(lib_path, self._libraries[lib_key])
            else:
                logger.warn("Wrong path in config for LibraryManager: Path %s does not exist", lib_path)
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
            #logger.debug(str(lib))
            if os.path.isdir(os.path.join(lib_path, lib)):
                if os.path.exists(os.path.join(os.path.join(lib_path, lib), Storage.STATEMACHINE_FILE)):
                    self.add_library(lib, lib_path, target_dict)
                else:
                    target_dict[lib] = {}
                    self.add_libraries_from_path(os.path.join(lib_path, lib), target_dict[lib])

    def add_library(self, lib, lib_path, target_dict):
        """
        Adds a single library path to the specified library dictionary.
        :param lib: the library to add
        :param lib_path: the path to the library specified in lib
        :param target_dict: the library dictionary to save the loaded library to
        :return:
        """
        self.storage.base_path = lib_path
        target_dict[lib] = os.path.join(lib_path, lib)  # self._storage.load_statemachine_from_yaml(os.path.join(lib_path, lib))

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
