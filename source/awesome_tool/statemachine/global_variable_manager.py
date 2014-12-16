"""
.. module:: global_variable_manager
   :platform: Unix, Windows
   :synopsis: A module to organize all global variables of the state machine

.. moduleauthor:: Sebastian Brunner

"""


from gtkmvc import Observable
from threading import Lock
from id_generator import *


class GlobalVariableManager(Observable):

    """A class for organizing all global variables of the state machine

    :ivar __global_variable_dictionary: the dictionary, where all global variables are stored
    :ivar __variable_locks: a dictionary that holds one mutex for each global variable
    :ivar __dictionary_lock: a mutex to prevent that the dictionary is written by two threads simultaneously
    :ivar __access_keys: a dictionary that holds an access key to each locked global variable

    """

    def __init__(self):
        Observable.__init__(self)
        self.__global_variable_dictionary = {}
        self.__variable_locks = {}
        self.__dictionary_lock = Lock()
        self.__access_keys = {}

    def set_variable(self, key, value):
        """Sets a global variable

        :param key: the key of the global variable to be set
        :param value: the new value of the global variable

        """
        self.__dictionary_lock.acquire()
        self.__variable_locks[key] = Lock()
        access_key = self.lock_variable(key)
        # --- variable locked
        self.__global_variable_dictionary[key] = value
        # --- release variable
        self.unlock_variable(key, access_key)
        self.__dictionary_lock.release()
        print "Variable %s was set to %s" % (key, str(value))

    def get_variable(self, key):
        """Fetches the value of a global variable

        :param key: the key of the global variable to be fechted

        """
        return_value = None
        access_key = self.lock_variable(key)
        return_value = self.__global_variable_dictionary[key]
        self.unlock_variable(key, access_key)
        return return_value

    def lock_variable(self, key):
        """Locks a global variable

        :param key: the key of the global variable to be locked

        """
        self.__variable_locks[key].acquire()
        access_key = global_variable_id_generator()
        self.__access_keys[key] = access_key
        return access_key

    def unlock_variable(self, key, access_key):
        """Unlocks a global variable

        :param key: the key of the global variable to be unlocked

        """
        if self.__access_keys[key] is access_key:
            self.__variable_locks[key].release()
        else:
            raise RuntimeError("Wrong access key for accessing global variable")

    def set_locked_variable(self, key, access_key, value):
        """Set an already locked global variable

        :param key: the key of the global variable to be set
        :param access_key: the access key to the already locked global variable
        :param value: the new value of the global variable

        """
        if self.__access_keys[key] is access_key:
            self.__global_variable_dictionary[key] = value
        else:
            raise RuntimeError("Wrong access key for accessing global variable")

    def get_locked_variable(self, key, access_key):
        """Returns the value of an global variable that is already locked

        :param key: the key of the global variable
        :param access_key: the access_key to the global variable that is already locked

        """
        if self.__access_keys[key] is access_key:
            return self.__global_variable_dictionary[key]
        else:
            raise RuntimeError("Wrong access key for accessing global variable")

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    #TODO: does the gtkmvc observer pattern work, when there is no property setter function given?
    @property
    def global_variable_dictionary(self):
        """Property for the _global_variable_dictionary field

        """
        return self.__global_variable_dictionary

    @property
    def variable_locks(self):
        """Property for the _variable_locks field

        """
        return self.__variable_locks

    @property
    def dictionary_lock(self):
        """Property for the _dictionary_lock field

        """
        return self.__dictionary_lock