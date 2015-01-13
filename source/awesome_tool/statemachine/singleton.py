"""
.. module:: singleton
   :platform: Unix, Windows
   :synopsis: A module to hold all singletons of the state machine

.. moduleauthor:: Sebastian Brunner


"""

from statemachine.global_variable_manager import GlobalVariableManager
from statemachine.external_modules.external_module_manager import ExternalModuleManager
from statemachine.library_manager import LibraryManager

"""This variable holds the global variable manager singleton

"""
global_variable_manager = GlobalVariableManager()

"""This variable holds the external module manager singleton

"""
external_module_manager = ExternalModuleManager()

"""This variable holds the library manager singleton

"""
library_manager = LibraryManager()