"""
.. module:: global_variable_manager
   :platform: Unix, Windows
   :synopsis: A module to organize all global variables of the state machine

.. moduleauthor:: Sebastian Brunner

"""


from gtkmvc import Observable
from threading import Lock
from awesome_tool.statemachine.id_generator import *

from awesome_tool.utils import log
logger = log.get_logger(__name__)
import copy


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
        self.__variable_references = {}

    @Observable.observed
    def set_variable(self, key, value, per_reference=False, access_key=None):
        """Sets a global variable

        :param key: the key of the global variable to be set
        :param value: the new value of the global variable

        """
        self.__dictionary_lock.acquire()
        unlock = True
        if self.variable_exist(key):
            if self.is_locked(key) and self.__access_keys[key] != access_key:
                raise RuntimeError("Wrong access key for accessing global variable")
            elif self.is_locked(key):
                unlock = False
            else:
                access_key = self.lock_variable(key)
        else:
            self.__variable_locks[key] = Lock()
            access_key = self.lock_variable(key)

        # --- variable locked
        if per_reference:
            self.__global_variable_dictionary[key] = value
            self.__variable_references[key] = True
        else:
            self.__global_variable_dictionary[key] = copy.deepcopy(value)
            self.__variable_references[key] = False
        # --- release variable

        if unlock:
            self.unlock_variable(key, access_key)
        self.__dictionary_lock.release()
        logger.debug("Global variable %s was set to %s" % (key, str(value)))

    def get_variable(self, key, per_reference=False, access_key=None):
        """Fetches the value of a global variable

        :param key: the key of the global variable to be fetched
        :return: The value stored at in the global variable key
        """
        if self.variable_exist(key):
            unlock = True
            if self.is_locked(key) and self.__access_keys[key] != access_key:
                raise RuntimeError("Wrong access key for accessing global variable")
            elif self.is_locked(key):
                unlock = False
            else:
                access_key = self.lock_variable(key)

            # --- variable locked
            if per_reference:
                if self.variable_can_be_referenced(key):
                    return_value = self.__global_variable_dictionary[key]
                else:
                    self.unlock_variable(key, access_key)
                    raise RuntimeError("Variable cannot be accessed by reference")
            else:
                return_value = copy.deepcopy(self.__global_variable_dictionary[key])
            # --- release variable

            if unlock:
                self.unlock_variable(key, access_key)
            return return_value
        else:
            raise AttributeError("Global variable %s does not exist!" % str(key))

    def variable_can_be_referenced(self, key):
        """Checks whether the value of the variable can be returned by reference

        :param str key: Name of the variable
        :return: True if value of variable can bbe returned by reference, False else
        """
        return key in self.__variable_references and self.__variable_references[key]

    @Observable.observed
    def delete_global_variable(self, key):
        """Deletes a global variable

        :param key: the key of the global variable to be deleted

        """
        self.__dictionary_lock.acquire()
        if key in self.__global_variable_dictionary:
            access_key = self.lock_variable(key)
            del self.__global_variable_dictionary[key]
            self.unlock_variable(key, access_key)
            del self.__variable_locks[key]
            del self.__variable_references[key]
        else:
            raise AttributeError("Global variable %s does not exist!" % str(key))
        self.__dictionary_lock.release()
        logger.debug("Global variable %s was deleted!" % str(key))

    @Observable.observed
    def lock_variable(self, key):
        """Locks a global variable

        :param key: the key of the global variable to be locked

        """
        if key in self.__variable_locks:
            self.__variable_locks[key].acquire()
            access_key = global_variable_id_generator()
            self.__access_keys[key] = access_key
            return access_key

    @Observable.observed
    def unlock_variable(self, key, access_key):
        """Unlocks a global variable

        :param key: the key of the global variable to be unlocked
        :param access_key: the access key to be able to unlock the global variable

        """
        if self.__access_keys[key] == access_key:
            if key in self.__variable_locks:
                self.__variable_locks[key].release()
            else:
                raise AttributeError("Global variable %s does not exist!" % str(key))
        else:
            raise RuntimeError("Wrong access key for accessing global variable")

    @Observable.observed
    def set_locked_variable(self, key, access_key, value):
        """Set an already locked global variable

        :param key: the key of the global variable to be set
        :param access_key: the access key to the already locked global variable
        :param value: the new value of the global variable

        """
        return self.set_variable(key, value, per_reference=False, access_key=access_key)

    def get_locked_variable(self, key, access_key):
        """Returns the value of an global variable that is already locked

        :param key: the key of the global variable
        :param access_key: the access_key to the global variable that is already locked

        """
        return self.get_variable(key, per_reference=False, access_key=access_key)

    def variable_exist(self, key):
        """Checks if a global variable exist

        :param key: the name of the global variable

        """
        return key in self.__global_variable_dictionary

    def locked_status_for_variable(self, key):
        """
        Returns the status of the lock of a global variable
        :param key: the unique key of the global variable
        :return:
        """
        if key in self.__variable_locks:
            return self.__variable_locks[key].locked()
        return False

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def global_variable_dictionary(self):
        """Property for the _global_variable_dictionary field

        """
        return copy.deepcopy(self.__global_variable_dictionary)

    def is_locked(self, key):
        """Check whether a variable is currently locked

        :param str key: The name of the variable
        :return: True if locked, False else
        """
        return self.locked_status_for_variable(key)
