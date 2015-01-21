"""
.. module:: external_module_manager
   :platform: Unix, Windows
   :synopsis: A module to organize all external modules of a state machine

.. moduleauthor:: Sebastian Brunner


"""


from statemachine.external_modules.external_module import ExternalModule
from gtkmvc import Observable
from utils import log
logger = log.get_logger(__name__)


class ExternalModuleManager(Observable):

    """A class for representing the external module manager which holds all external modules of the state machine.

    It inherits from Observable to make a change of its fields observable.

    :ivar _external_modules: the list of all external modules

    """

    def __init__(self):
        Observable.__init__(self)
        self._external_modules = {}

    @Observable.observed
    def add_external_module(self, module):
        """Adds a new external module to the list of external modules

        """
        self._external_modules[module.name] = module
        logger.debug("External module %s was added to the manager" % (str(module.name)))


#########################################################################
# Properties for all class fields that must be observed by gtkmvc
#########################################################################

    @property
    def external_modules(self):
        """Property for the _external_modules field

        """
        return self._external_modules

    @external_modules.setter
    @Observable.observed
    def external_modules(self, external_modules):
        if not isinstance(external_modules, dict):
            raise TypeError("external_modules must be of type dict")
        self._external_modules = external_modules
