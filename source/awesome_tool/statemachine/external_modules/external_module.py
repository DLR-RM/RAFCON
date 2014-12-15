"""
.. module:: external_module
   :platform: Unix, Windows
   :synopsis: A module to represent an external module

.. moduleauthor:: Sebastian Brunner


"""

import os
import sys
import types
import inspect
from enum import Enum
from gtkmvc import Observable

from statemachine.id_generator import *
from utils import log
logger = log.get_logger(__name__)


def class_by_name(module, class_name):
    try:
        identifier = getattr(module, class_name)
    except AttributeError:
        raise NameError("%s doesn't exist." % class_name)
    if isinstance(identifier, (types.ClassType, types.TypeType)):
        return identifier
    raise TypeError("%s is not a class." % class_name)


EMStatus = Enum('EM_STATUS', 'STOPPED STARTED PAUSED')


class ExternalModule(Observable):

    """A class for representing an external module of the statemachine

    It inherits from Observable to make a change of its fields observable.

    :ivar _name: the name of the external module
    :ivar _module_name: the name of the module to import
    :ivar _class_name: the class of the module, of which an instance will be created during
                        :func:`ExternalModule.connect`
    :ivar _module_id: the generated id of the module
    :ivar _status: the status of the external module (stopped, started, paused)
    :ivar _list_params: the *args parameter that can be passed to the constructor of :attr:`_class_name`
    :ivar _dict_params: the *kwargs parameter that can be passed to the constructor of :attr:`_class_name`
    :ivar _tried_load: specifies if :func:`ExternalModule.load` was called yet
    """

    def __init__(self, name=None, module_name=None, class_name=None, *args, **kwargs):
        Observable.__init__(self)
        self._name = name
        self._module_name = module_name
        self._class_name = class_name
        self._module_id = generate_external_module_id()
        self._status = EMStatus.STOPPED
        self._list_params = args
        self._dict_params = kwargs
        self._tried_load = False

        self.__start_method = None
        self.__stop_method = None
        self.__pause_method = None
        self.__module_loaded = False
        self.__class = None
        self.__instance = None

    def load(self):
        """loads and imports the external module

        """
        logger.debug("load external module %s" % str(self._module_name))
        if not self.__class:
            self._tried_load = True
            #try:
            if not self._module_name in sys.modules:
                __import__(self._module_name)
            else:
                reload(sys.modules[self._module_name])
            mod = sys.modules[self._module_name]
            self.__class = class_by_name(mod, self._class_name)
            #except:
            #    raise IOError("External module could not be loaded")

    def reload(self):
        """reloads the external module

        """
        if not self.__class:
            self.load()
        else:
            mod = sys.modules[self._module_name]
            reload(mod)
            self.__class = class_by_name(mod, self._class_name)

    def connect(self, list_arguments):
        """creates an instance of the class given by self._class_name and connects the default functions of the external
        module class to the custom functions of external module

        """
        if not self.__module_loaded:
            self.load()
            self.__module_loaded = True

        logger.debug("connect to the external module %s" % str(self._module_name))
        if not isinstance(list_arguments, list):
            raise TypeError("list_arguments should be of type list")
        self.__instance = self.__class(*self._list_params, **self._dict_params)
        #try:
        self.__start_method = self.__class.start
        self.__stop_method = self.__class.stop
        self.__pause_method = self.__class.pause
        if not inspect.ismethod(self.__start_method):
            self.__start_method = None
        if not inspect.ismethod(self.__stop_method):
            self.__stop_method = None
        if not inspect.ismethod(self.__pause_method):
            self.__pause_method = None
        #except AttributeError:
        #    pass

    def disconnect(self):
        """stops the external module and disconnect the default functions

        """
        self.stop()
        del self.__instance
        self.__instance = None
        self.__start_method = None
        self.__stop_method = None
        self.__pause_method = None

    def start(self):
        """starts the external module

        """
        if self.__start_method:
            self.__start_method(self.__instance)
            self._status = EMStatus.STARTED

    def pause(self):
        """pauses the external module

        """
        if self.__pause_method:
            self.__pause_method(self.__instance)
            self._status = EMStatus.PAUSED

    def stop(self):
        """stops the external module

        """
        if self.__stop_method:
            self.__stop_method(self.__instance)
            self._status = EMStatus.STOPPED

#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def name(self):
        """Property for the _name field

        """
        return self._name

    @name.setter
    @Observable.observed
    def name(self, name):
        if not isinstance(name, str):
            raise TypeError("name must be of type dict")
        self._name = name

    @property
    def status(self):
        """Property for the _status field

        """
        return self._status

    @status.setter
    @Observable.observed
    def status(self, status):
        if not isinstance(status, dict):
            raise TypeError("external_modules must be of type dict")
        self._status = status

    @property
    def instance(self):
        """Property for the __instance field

        """
        return self.__instance

    @instance.setter
    @Observable.observed
    def instance(self, instance):
        self.__instance = instance