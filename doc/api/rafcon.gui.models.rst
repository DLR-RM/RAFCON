MVC Models: ``rafcon.gui.models``
=================================

This package contains all models of the MVC architecture.

The models hold the data for, which are shown in the views. These models typically hold an element of the core, which
is observable. For example, the StateModel holds a reference to a State class object from the core. If the core
element changes, the model recognizes these changes and forwards the notification to the controllers, if they observe
the changed model property.

.. contents::
    :backlinks: top

AbstractStateModel (in abstract_state)
--------------------------------------

.. automodule:: rafcon.gui.models.abstract_state

StateModel (in state)
---------------------

.. automodule:: rafcon.gui.models.state


ContainerStateModel (in container_state)
----------------------------------------

.. automodule:: rafcon.gui.models.container_state


LibraryStateModel (in library_state)
------------------------------------

.. automodule:: rafcon.gui.models.library_state


StateElementModel (in state_element)
------------------------------------

.. automodule:: rafcon.gui.models.state_element


TransitionModel (in transition)
-------------------------------

.. automodule:: rafcon.gui.models.transition


DataFlowModel (in data_flow)
----------------------------

.. automodule:: rafcon.gui.models.data_flow


DataPortModel (in data_port)
----------------------------

.. automodule:: rafcon.gui.models.data_port


ScopedVariableModel (in scoped_variable)
----------------------------------------

.. automodule:: rafcon.gui.models.scoped_variable

Selection (in selection)
------------------------
.. automodule:: rafcon.gui.models.selection


OutcomeModel (in outcome)
-------------------------

.. automodule:: rafcon.gui.models.outcome


StateMachineModel (in state_machine)
------------------------------------

.. automodule:: rafcon.gui.models.state_machine

ModificationsHistoryModel (in modification_history)
---------------------------------------------------

.. automodule:: rafcon.gui.models.modification_history

AutoBackupModel (in auto_backup)
--------------------------------

.. automodule:: rafcon.gui.models.auto_backup

StateMachineManagerModel (in state_machine_manager)
---------------------------------------------------

.. automodule:: rafcon.gui.models.state_machine_manager


GlobalVariableManagerModel (in global_variable_manager)
-------------------------------------------------------

.. automodule:: rafcon.gui.models.global_variable_manager


LibraryManagerModel (in library_manager)
----------------------------------------

.. automodule:: rafcon.gui.models.library_manager

StateMachineExecutionEngineModel (in state_machine_execution_engine)
--------------------------------------------------------------------

.. automodule:: rafcon.gui.models.state_machine_execution_engine
