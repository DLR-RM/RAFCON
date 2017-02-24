The core: ``statemachine``
==========================

.. contents::
    :backlinks: top

Subpackages
-----------

.. toctree::

    rafcon.core.execution
    rafcon.core.state_elements
    rafcon.core.states
    rafcon.core.storage

config
------
.. automodule:: rafcon.core.config
   :synopsis: Config module to specify global constants

constants
---------
.. automodule:: rafcon.core.constants
   :synopsis: A module which holds all global enumerations for the state machine

custom_exceptions
-----------------
.. automodule:: rafcon.core.custom_exceptions
   :synopsis: A module that holds all custom exceptions for the state machine core

decorators
----------
.. automodule:: rafcon.core.decorators
   :synopsis: A module to hold all decorators needed for the RAFCON core

global_variable_manager
-----------------------
.. automodule:: rafcon.core.global_variable_manager
   :synopsis: A module to organize all global variables of the state machine

id_generator
------------
.. automodule:: rafcon.core.id_generator
   :synopsis: A module to generate different kinds of state machine ids

interface
---------
.. automodule:: rafcon.core.interface
   :synopsis: This is a interface for user input. In absence of a GUI the input is read from stdin.
                If e.g. a gtk GUI is implemented the function can be replaced with appropriate dialog boxes.

library_manager
---------------
.. automodule:: rafcon.core.library_manager
   :synopsis: A module to handle all libraries for a state machine

script
------
.. automodule:: rafcon.core.script
   :synopsis: A module to represent the script file for each state in a state machine

singleton
---------
.. automodule:: rafcon.core.singleton
   :synopsis: A module to hold all singletons of the state machine

start
-----
.. automodule:: rafcon.core.start
   :synopsis: A module to start arbitrary state machines without the GUI and several configurations options

state_machine
-------------
.. automodule:: rafcon.core.state_machine
   :synopsis: A module to organize a state machine with all its main components

state_machine_manager
---------------------
.. automodule:: rafcon.core.state_machine_manager
   :synopsis: A module to organize a open state machine with all its main components
