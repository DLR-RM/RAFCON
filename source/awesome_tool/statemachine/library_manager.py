"""
.. module:: library_manager
   :platform: Unix, Windows
   :synopsis: A module to handle all libraries for a statemachine

.. moduleauthor:: Sebastian Brunner


"""


from gtkmvc import Observable
import os

from utils import log
logger = log.get_logger(__name__)
import config
from statemachine.storage.storage import Storage


class LibraryManager(Observable):

    """This class manages all libraries specified in config.py.
    Libraries are essentially just (reusable) state machines.

    :ivar _libraries: a dictionary to hold  all libraries
    """

    def __init__(self):
        Observable.__init__(self)
        self._libraries = {}
        logger.debug("Initializing Storage object ...")
        self._storage = Storage("../")

    # cannot be done in the __init__ function as the the library_manager can be compiled and executed by singleton.py
    # before the state*.pys are loaded
    def initialize(self):
        logger.debug("Initializing LibraryManager: Loading libraries ... ")
        for lib_path in config.LIBRARY_PATHS:
            self.add_libraries_from_path(lib_path, self._libraries)
        logger.debug("Initialization of LibraryManager done.")

    def add_libraries_from_path(self, lib_path, target_dict):
        for lib in os.listdir(lib_path):
            if os.path.isdir(os.path.join(lib_path, lib)):
                if os.path.exists(os.path.join(os.path.join(lib_path, lib), Storage.STATEMACHINE_FILE)):
                    self.add_library(lib, lib_path, target_dict)
                else:
                    target_dict[lib] = {}
                    self.add_libraries_from_path(os.path.join(lib_path, lib), target_dict[lib])

    def add_library(self, lib, lib_path, target_dict):
        #print lib
        self._storage.base_path = lib_path
        target_dict[lib] = self._storage.load_statemachine_from_yaml(os.path.join(lib_path, lib))
        #print self.libraries[lib]

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
