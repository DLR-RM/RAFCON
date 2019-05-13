# Copyright (C) 2014-2018 DLR
#
# All rights reserved. This program and the accompanying materials are made
# available under the terms of the Eclipse Public License v1.0 which
# accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v10.html
#
# Contributors:
# Annika Wollschlaeger <annika.wollschlaeger@dlr.de>
# Benno Voggenreiter <benno.voggenreiter@dlr.de>
# Franz Steinmetz <franz.steinmetz@dlr.de>
# Lukas Becker <lukas.becker@dlr.de>
# Mahmoud Akl <mahmoud.akl@dlr.de>
# Rico Belder <rico.belder@dlr.de>
# Sebastian Brunner <sebastian.brunner@dlr.de>

"""
.. module:: global_variable_manager
   :synopsis: A module to organize all global variables of the state machine

"""

from builtins import str
import time
import copy
from gtkmvc3.observable import Observable
from threading import Lock, currentThread, RLock
from rafcon.core.id_generator import *

from rafcon.utils.type_helpers import type_inherits_of_type
from rafcon.utils import log
from rafcon.utils import type_helpers
logger = log.get_logger(__name__)


class GlobalVariableManager(Observable):
    """A class for organizing all global variables of the state machine

    :ivar __global_variable_dictionary: the dictionary, where all global variables are stored
    :ivar __variable_locks: a dictionary that holds one mutex for each global variable
    :ivar __global_lock: a mutex to prevent that the dictionary is written by two threads simultaneously
    :ivar __access_keys: a dictionary that holds an access key to each locked global variable
    :ivar __variable_references: a dictionary that stores whether a variable can be returned by reference or not
    """

    def __init__(self):
        Observable.__init__(self)
        self.__global_variable_dictionary = {}
        self.__global_variable_type_dictionary = {}
        self.__variable_locks = {}
        self.__global_lock = RLock()
        self.__access_keys = {}
        self.__variable_references = {}

    @Observable.observed
    def set_variable(self, key, value, per_reference=False, access_key=None, data_type=None):
        """Sets a global variable

        :param key: the key of the global variable to be set
        :param value: the new value of the global variable
        :param per_reference: a flag to decide if the variable should be stored per reference or per value
        :param access_key: if the variable was explicitly locked with the  rafcon.state lock_variable
        :raises exceptions.RuntimeError: if a wrong access key is passed
        """
        key = str(key)  # Ensure that we have the same string type for all keys (under Python2 and 3!)
        if self.variable_exist(key):
            if data_type is None:
                data_type = self.__global_variable_type_dictionary[key]
        else:
            if data_type is None:
                data_type = type(None)
        assert isinstance(data_type, type)
        self.check_value_and_type(value, data_type)

        with self.__global_lock:
            unlock = True
            if self.variable_exist(key):
                if self.is_locked(key) and not access_key:  # case: locked, no access key; thus, wait until I get lock
                    access_key = self.lock_variable(key, block=True)
                elif self.is_locked(key) and self.__access_keys[key] != access_key:  # case: locked, but wrong access key
                    raise RuntimeError("Wrong access key for accessing global variable")
                elif self.is_locked(key):  # case: locked and correct access key
                    unlock = False
                else:  # case not locked
                    access_key = self.lock_variable(key, block=True)
            else:
                self.__variable_locks[key] = Lock()
                access_key = self.lock_variable(key, block=True)

            # --- variable locked
            if per_reference:
                self.__global_variable_dictionary[key] = value
                self.__global_variable_type_dictionary[key] = data_type
                self.__variable_references[key] = True
            else:
                self.__global_variable_dictionary[key] = copy.deepcopy(value)
                self.__global_variable_type_dictionary[key] = data_type
                self.__variable_references[key] = False
            # --- release variable

            if unlock:
                self.unlock_variable(key, access_key)

        logger.debug("Global variable '{}' was set to value '{}' with type '{}'".format(key, value, data_type.__name__))

    def get_variable(self, key, per_reference=None, access_key=None, default=None):
        """Fetches the value of a global variable

        :param key: the key of the global variable to be fetched
        :param bool per_reference: a flag to decide if the variable should be stored per reference or per value
        :param access_key: if the variable was explicitly locked with the  rafcon.state lock_variable
        :param default: a value to be returned if the key does not exist
        :return: The value stored at in the global variable key
        :raises exceptions.RuntimeError: if a wrong access key is passed or the variable cannot be accessed by reference
        """
        key = str(key)
        if self.variable_exist(key):
            unlock = True
            if self.is_locked(key):
                if self.__access_keys[key] == access_key:
                    unlock = False
                else:
                    if not access_key:
                        access_key = self.lock_variable(key, block=True)
                    else:
                        raise RuntimeError("Wrong access key for accessing global variable")
            else:
                access_key = self.lock_variable(key, block=True)

            # --- variable locked
            if self.variable_can_be_referenced(key):
                if per_reference or per_reference is None:
                    return_value = self.__global_variable_dictionary[key]
                else:
                    return_value = copy.deepcopy(self.__global_variable_dictionary[key])
            else:
                if per_reference:
                    self.unlock_variable(key, access_key)
                    raise RuntimeError("Variable cannot be accessed by reference")
                else:
                    return_value = copy.deepcopy(self.__global_variable_dictionary[key])
            # --- release variable

            if unlock:
                self.unlock_variable(key, access_key)
            return return_value
        else:
            # logger.warning("Global variable '{0}' not existing, returning default value".format(key))
            return default

    def variable_can_be_referenced(self, key):
        """Checks whether the value of the variable can be returned by reference

        :param str key: Name of the variable
        :return: True if value of variable can be returned by reference, False else
        """
        key = str(key)
        return key in self.__variable_references and self.__variable_references[key]

    @Observable.observed
    def delete_variable(self, key):
        """Deletes a global variable

        :param key: the key of the global variable to be deleted
        :raises exceptions.AttributeError:  if the global variable does not exist
        """
        key = str(key)
        if self.is_locked(key):
            raise RuntimeError("Global variable is locked")

        with self.__global_lock:
            if key in self.__global_variable_dictionary:
                access_key = self.lock_variable(key, block=True)
                del self.__global_variable_dictionary[key]
                self.unlock_variable(key, access_key)
                del self.__variable_locks[key]
                del self.__variable_references[key]
            else:
                raise AttributeError("Global variable %s does not exist!" % str(key))

        logger.debug("Global variable %s was deleted!" % str(key))

    @Observable.observed
    def lock_variable(self, key, block=False):
        """Locks a global variable

        :param key: the key of the global variable to be locked
        :param block: a flag to specify if to wait for locking the variable in blocking mode
        """
        key = str(key)
        # watch out for releasing the __dictionary_lock properly
        try:
            if key in self.__variable_locks:
                # acquire without arguments is blocking
                lock_successful = self.__variable_locks[key].acquire(False)
                if lock_successful or block:
                    if (not lock_successful) and block:  # case: lock could not be acquired => wait for it as block=True
                        duration = 0.
                        loop_time = 0.1
                        while not self.__variable_locks[key].acquire(False):
                            time.sleep(loop_time)
                            duration += loop_time
                            if int(duration*10) % 20 == 0:
                                # while loops informs the user about long locked variables
                                logger.verbose("Variable '{2}' is locked and thread {0} waits already {1} seconds to "
                                               "access it.".format(currentThread(), duration, key))
                    access_key = global_variable_id_generator()
                    self.__access_keys[key] = access_key
                    return access_key
                else:
                    logger.warning("Global variable {} already locked".format(str(key)))
                    return False
            else:
                logger.error("Global variable key {} does not exist".format(str(key)))
                return False
        except Exception as e:
            logger.error("Exception thrown: {}".format(str(e)))
            return False

    @Observable.observed
    def unlock_variable(self, key, access_key, force=False):
        """Unlocks a global variable

        :param key: the key of the global variable to be unlocked
        :param access_key: the access key to be able to unlock the global variable
        :param force: if the variable should be unlocked forcefully
        :raises exceptions.AttributeError: if the global variable does not exist
        :raises exceptions.RuntimeError: if the wrong access key is passed
        """
        key = str(key)
        if self.__access_keys[key] == access_key or force:
            if key in self.__variable_locks:
                if self.is_locked(key):
                    self.__variable_locks[key].release()
                    return True
                else:
                    logger.error("Global variable {} is not locked, thus cannot unlock it".format(str(key)))
                    return False
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
        key = str(key)
        return key in self.__global_variable_dictionary

    variable_exists = variable_exist

    def data_type_exist(self, key):
        """Checks if a global variable exist

        :param key: the name of the global variable
        """
        key = str(key)
        return key in self.__global_variable_type_dictionary

    def is_locked(self, key):
        """Returns the status of the lock of a global variable

        :param key: the unique key of the global variable
        :return:
        """
        key = str(key)
        if key in self.__variable_locks:
            return self.__variable_locks[key].locked()
        return False

    def get_all_keys_starting_with(self, start_key):
        """ Returns all keys, which start with a certain pattern defined in :param start_key.

        :param start_key: The start pattern to search all keys for.
        :return:
        """
        start_key = str(start_key)
        output_list = []
        if len(self.__global_variable_dictionary) == 0:
            return output_list
        for g_key in self.__global_variable_dictionary.keys():
            # string comparison
            if g_key and start_key in g_key:
                output_list.append(g_key)
        return output_list

#########################################################################
# Properties for all class fields that must be observed by gtkmvc3
#########################################################################

    @property
    def global_variable_dictionary(self):
        """Property for the _global_variable_dictionary field"""
        dict_copy = {}
        for key, value in self.__global_variable_dictionary.items():
            if key in self.__variable_references and self.__variable_references[key]:
                dict_copy[key] = value
            else:
                dict_copy[key] = copy.deepcopy(value)

        return dict_copy

    def get_all_keys(self):
        """Returns all variable names in the GVM

        :return: Keys of all variables
        """
        return list(self.__global_variable_dictionary.keys())

    def get_representation(self, key):
        key = str(key)
        if not self.variable_exist(key):
            return None
        return self.__global_variable_dictionary[key]

    def get_data_type(self, key):
        key = str(key)
        if not self.data_type_exist(key):
            return None
        return self.__global_variable_type_dictionary[key]

    @staticmethod
    def check_value_and_type(value, data_type):
        """Checks if a given value is of a specific type

        :param value: the value to check
        :param data_type: the type to be checked upon
        :return:
        """
        if value is not None and data_type is not type(None):
            # if not isinstance(value, data_type):
            if not type_inherits_of_type(data_type, type(value)):
                raise TypeError(
                    "Value: '{0}' is not of data type: '{1}', value type: {2}".format(value, data_type, type(value)))
